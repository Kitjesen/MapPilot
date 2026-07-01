from __future__ import annotations

import json
import os
import sys
import types
from pathlib import Path

import pytest

pytestmark = [pytest.mark.sim]


class _FakeGateway:
    def __init__(self, run_result: bool):
        self._defer_server = False
        self._run_result = run_result
        self.run_server_called = False

    def _run_server(self) -> bool:
        self.run_server_called = True
        return self._run_result


class _FakeSystem:
    def __init__(self, gateway: _FakeGateway):
        self.gateway = gateway
        self.modules = {"GatewayModule": gateway}
        self.build_transport = "not-built"
        self.started = False
        self.stopped = False

    def get_module(self, name: str):
        if name != "GatewayModule":
            raise KeyError(name)
        return self.gateway

    def start(self) -> None:
        assert self.gateway._defer_server is True
        self.started = True

    def stop(self) -> None:
        self.stopped = True


class _FakeBuilder:
    def __init__(self, system: _FakeSystem):
        self._system = system

    def build(self, transport=None) -> _FakeSystem:
        self._system.build_transport = transport
        return self._system


def _install_cli_harness(monkeypatch, tmp_path, system: _FakeSystem) -> dict:
    import cli.main as main_mod
    import core.blueprints.products as products_mod

    calls = {"full_stack": [], "product": []}

    def _fake_full_stack_blueprint(**kwargs):
        calls["full_stack"].append(dict(kwargs))
        return _FakeBuilder(system)

    def _fake_thunder_blueprint(config=None, **overrides):
        resolved = dict(config or {})
        resolved.update(overrides)
        calls["product"].append(resolved)
        return _FakeBuilder(system)

    fake_full_stack = types.ModuleType("core.blueprints.full_stack")
    fake_full_stack.full_stack_blueprint = _fake_full_stack_blueprint
    monkeypatch.setitem(sys.modules, "core.blueprints.full_stack", fake_full_stack)
    monkeypatch.setattr(products_mod, "thunder_blueprint", _fake_thunder_blueprint)

    fake_ros2 = types.ModuleType("compat.ros2.context")
    fake_ros2.shutdown_shared_executor = lambda: None
    monkeypatch.setitem(sys.modules, "compat.ros2.context", fake_ros2)

    fake_service_manager = types.ModuleType("core.service_manager")
    fake_service_manager.get_service_manager = lambda: types.SimpleNamespace(
        _started=[]
    )
    monkeypatch.setitem(sys.modules, "core.service_manager", fake_service_manager)

    monkeypatch.setattr(main_mod, "setup_logging", lambda *args, **kwargs: tmp_path)
    monkeypatch.setattr(main_mod, "preflight", lambda *args, **kwargs: None)
    monkeypatch.setattr(main_mod, "health_check", lambda _system: True)
    monkeypatch.setattr(main_mod, "kill_residual_ports", lambda _cfg: None)
    monkeypatch.setattr(main_mod, "print_banner", lambda *args, **kwargs: None)
    monkeypatch.setattr(main_mod, "save_run_state", lambda *args, **kwargs: None)
    monkeypatch.setattr(main_mod, "update_run_state", lambda *args, **kwargs: None)
    monkeypatch.setattr(main_mod, "clear_run_state", lambda: None)
    monkeypatch.setattr(main_mod.signal, "signal", lambda *args, **kwargs: None)
    return calls


def test_cli_shutdown_uses_compat_ros_context() -> None:
    source = Path("cli/main.py").read_text(encoding="utf-8-sig")

    assert "from core.ros2_context import shutdown_shared_executor" not in source
    assert "from compat.ros2.context import shutdown_shared_executor" in source


def test_cli_help_shows_product_endpoint_aliases_not_legacy_board_names(
    monkeypatch,
    capsys,
) -> None:
    import cli.main as main_mod

    monkeypatch.setattr(sys, "argv", ["lingtu.py", "--help"])

    with pytest.raises(SystemExit) as exc:
        main_mod.main()

    out = capsys.readouterr().out
    assert exc.value.code == 0
    assert "--endpoint thunder-field" in out
    assert "--endpoint thunder-lite" in out
    assert "real_s100p" not in out
    assert "s100p" not in out.lower()


def test_cli_accepts_legacy_endpoint_alias_without_showing_it(
    monkeypatch,
    capsys,
) -> None:
    import cli.main as main_mod

    monkeypatch.setattr(
        sys,
        "argv",
        ["lingtu.py", "runtime-spec", "thunder-nav", "--endpoint", "real_s100p", "--json"],
    )

    main_mod.main()

    payload = json.loads(capsys.readouterr().out)
    assert payload["ok"] is True
    assert payload["spec"]["endpoint"] == "thunder_field"
    assert payload["spec"]["robot_preset"] == "thunder"


def test_no_repl_exits_nonzero_when_gateway_server_returns_false(
    monkeypatch, tmp_path
):
    import cli.main as main_mod

    gateway = _FakeGateway(run_result=False)
    system = _FakeSystem(gateway)
    _install_cli_harness(monkeypatch, tmp_path, system)
    monkeypatch.setattr(sys, "argv", ["lingtu.py", "stub", "--no-repl"])

    with pytest.raises(SystemExit) as exc:
        main_mod.main()

    assert exc.value.code == 1
    assert gateway._defer_server is True
    assert gateway.run_server_called is True
    assert system.started is True
    assert system.stopped is True


def test_no_repl_clean_gateway_shutdown_exits_zero(monkeypatch, tmp_path):
    import cli.main as main_mod

    gateway = _FakeGateway(run_result=True)
    system = _FakeSystem(gateway)
    calls = _install_cli_harness(monkeypatch, tmp_path, system)
    monkeypatch.setattr(sys, "argv", ["lingtu.py", "stub", "--no-repl"])

    main_mod.main()

    assert len(calls["full_stack"]) == 1
    assert calls["product"] == []
    assert gateway._defer_server is True
    assert gateway.run_server_called is True
    assert system.started is True
    assert system.stopped is True


def test_in_process_profile_overrides_stale_runtime_env(monkeypatch, tmp_path, capsys):
    import cli.main as main_mod

    monkeypatch.setenv("LINGTU_PROFILE", "explore")
    monkeypatch.setenv("LINGTU_ENDPOINT", "mujoco_live")
    monkeypatch.setenv("LINGTU_DATA_SOURCE", "mujoco_fastlio2_live")
    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "mujoco_fastlio2_live")
    monkeypatch.setenv("LINGTU_COMMAND_SINK", "mujoco_velocity_adapter")
    monkeypatch.setenv("LINGTU_SIMULATION_ONLY", "1")

    gateway = _FakeGateway(run_result=True)
    system = _FakeSystem(gateway)
    calls = _install_cli_harness(monkeypatch, tmp_path, system)
    saved_state = {}

    def _capture_run_state(profile_name, cfg, log_dir, **kwargs):
        saved_state["profile"] = profile_name
        saved_state["cfg"] = cfg
        saved_state["log_dir"] = log_dir
        saved_state.update(kwargs)

    monkeypatch.setattr(main_mod, "save_run_state", _capture_run_state)
    monkeypatch.setattr(sys, "argv", ["lingtu.py", "nav", "--no-repl"])

    main_mod.main()

    out = capsys.readouterr().out
    assert "Runtime:  endpoint=thunder_field data_source=thunder_field" in out
    assert (
        "Topic frames: lidar_scan=lidar_link imu=lidar_link "
        "odometry=odom,map registered_cloud=body "
        "map_cloud=map global_path=map local_path=map,odom,body cmd_vel=body"
    ) in out
    assert "Flow stages: endpoint_adapter[endpoint_adapter|native_to_canonical]" in out
    assert "global_planning[lingtu_navigation_or_pct|map]" in out
    assert "command_boundary[cmd_vel_mux_to_endpoint_sink|body_twist]" in out
    assert os.environ["LINGTU_PROFILE"] == "nav"
    assert os.environ["LINGTU_ENDPOINT"] == "thunder_field"
    assert os.environ["LINGTU_DATA_SOURCE"] == "thunder_field"
    assert os.environ["LINGTU_MODULE_TRANSPORT"] == "local"
    assert os.environ["LINGTU_ENDPOINT_TRANSPORT"] == "lcm"
    assert os.environ["LINGTU_RUNTIME_CONTRACT"] == "thunder_field"
    assert os.environ["LINGTU_COMMAND_SINK"] == "hardware_driver_after_cmd_vel_mux"
    assert os.environ["LINGTU_SIMULATION_ONLY"] == "0"
    assert saved_state["runtime"]["endpoint"] == "thunder_field"
    assert saved_state["runtime"]["data_source"] == "thunder_field"
    assert saved_state["runtime"]["runtime_contract"] == "thunder_field"
    assert saved_state["runtime"]["module_transport"] == "local"
    assert saved_state["runtime"]["endpoint_transport"] == "lcm"
    assert saved_state["runtime"]["command_sink"] == "hardware_driver_after_cmd_vel_mux"
    assert saved_state["runtime"]["validation"] == {
        "ok": True,
        "blockers": [],
        "warnings": ["product semantic override: slam_profile localizer -> bridge"],
    }
    assert saved_state["runtime"]["resolved_runtime_data_flow"][-1]["outputs"] == [
        "hardware_driver_after_cmd_vel_mux",
    ]
    assert len(calls["product"]) == 1
    assert calls["product"][0]["robot"] == "thunder"
    assert calls["product"][0]["slam_profile"] == "bridge"
    assert calls["full_stack"] == []
    assert system.started is True


def test_thunder_nav_alias_runs_canonical_nav_profile(monkeypatch, tmp_path):
    import cli.main as main_mod

    gateway = _FakeGateway(run_result=True)
    system = _FakeSystem(gateway)
    calls = _install_cli_harness(monkeypatch, tmp_path, system)
    saved_state = {}

    def _capture_run_state(profile_name, cfg, log_dir, **kwargs):
        saved_state["profile"] = profile_name
        saved_state["cfg"] = cfg
        saved_state["log_dir"] = log_dir
        saved_state.update(kwargs)

    monkeypatch.setattr(main_mod, "save_run_state", _capture_run_state)
    monkeypatch.setattr(sys, "argv", ["lingtu.py", "thunder-nav", "--no-repl"])

    main_mod.main()

    assert os.environ["LINGTU_PROFILE"] == "nav"
    assert saved_state["profile"] == "nav"
    assert saved_state["runtime"]["endpoint"] == "thunder_field"
    assert saved_state["runtime"]["endpoint_transport"] == "lcm"
    assert saved_state["cfg"]["robot"] == "thunder"
    assert len(calls["product"]) == 1
    assert calls["product"][0]["robot"] == "thunder"
    assert calls["full_stack"] == []
    assert system.started is True


def test_cli_module_transport_override_reaches_runtime_build(
    monkeypatch,
    tmp_path,
):
    import cli.main as main_mod
    import core.blueprints.profile_builder as builder_mod

    gateway = _FakeGateway(run_result=True)
    system = _FakeSystem(gateway)
    calls = _install_cli_harness(monkeypatch, tmp_path, system)
    sentinel_transport = object()

    def fake_module_transport_for_resolved_config(config):
        assert config["module_transport"] == "lcm"
        return sentinel_transport

    monkeypatch.setattr(
        builder_mod,
        "module_transport_for_resolved_config",
        fake_module_transport_for_resolved_config,
    )
    monkeypatch.setattr(
        sys,
        "argv",
        ["lingtu.py", "thunder-nav", "--module-transport", "lcm", "--no-repl"],
    )

    main_mod.main()

    assert os.environ["LINGTU_MODULE_TRANSPORT"] == "lcm"
    assert os.environ["LINGTU_ENDPOINT_TRANSPORT"] == "lcm"
    assert calls["product"][0]["module_transport"] == "lcm"
    assert system.build_transport is sentinel_transport
    assert system.started is True


@pytest.mark.sim
def test_external_simulation_profile_runs_relative_launcher(monkeypatch):
    import subprocess

    import cli.main as main_mod

    captured = {}

    def _fake_run(cmd, *, cwd, env, check):
        captured["cmd"] = cmd
        captured["cwd"] = cwd
        captured["env"] = env
        captured["check"] = check
        return types.SimpleNamespace(returncode=0)

    monkeypatch.setattr(subprocess, "run", _fake_run)
    monkeypatch.setattr(sys, "argv", ["lingtu.py", "sim_mujoco_live", "status"])

    main_mod.main()

    assert captured["cmd"] == [
        "bash",
        "sim/scripts/launch_mujoco_fastlio2_live.sh",
        "status",
    ]
    assert (Path(captured["cwd"]) / "lingtu.py").exists()
    assert captured["env"]["LINGTU_PROFILE"] == "sim_mujoco_live"
    assert captured["env"]["LINGTU_RUNTIME_CONTRACT"] == "mujoco_fastlio2_live"
    assert captured["check"] is False


@pytest.mark.sim
def test_pct_mujoco_profile_defaults_to_pct_video_launcher(monkeypatch):
    import subprocess

    import cli.main as main_mod

    captured = {}

    def _fake_run(cmd, *, cwd, env, check):
        captured["cmd"] = cmd
        captured["cwd"] = cwd
        captured["env"] = env
        captured["check"] = check
        return types.SimpleNamespace(returncode=0)

    monkeypatch.setattr(subprocess, "run", _fake_run)
    monkeypatch.setattr(sys, "argv", ["lingtu.py", "sim_mujoco_pct_live"])

    main_mod.main()

    assert captured["cmd"] == [
        "bash",
        "sim/scripts/launch_mujoco_fastlio2_live.sh",
        "pct-moving-obstacle-video",
    ]
    assert (Path(captured["cwd"]) / "lingtu.py").exists()
    assert captured["env"]["LINGTU_PROFILE"] == "sim_mujoco_pct_live"
    assert captured["env"]["LINGTU_ENDPOINT"] == "mujoco_live"
    assert captured["env"]["LINGTU_DATA_SOURCE"] == "mujoco_fastlio2_live"
    assert captured["env"]["LINGTU_ENDPOINT_TRANSPORT"] == "local"
    assert captured["env"]["LINGTU_RUNTIME_CONTRACT"] == "mujoco_fastlio2_live"
    assert captured["check"] is False


@pytest.mark.sim
def test_product_task_endpoint_routes_to_mujoco_live_launcher(monkeypatch):
    import subprocess

    import cli.main as main_mod

    captured = {}

    def _fake_run(cmd, *, cwd, env, check):
        captured["cmd"] = cmd
        captured["cwd"] = cwd
        captured["env"] = env
        captured["check"] = check
        return types.SimpleNamespace(returncode=0)

    monkeypatch.setattr(subprocess, "run", _fake_run)
    monkeypatch.setattr(
        sys,
        "argv",
        ["lingtu.py", "explore", "--endpoint", "mujoco_live", "--record"],
    )

    main_mod.main()

    assert captured["cmd"] == [
        "bash",
        "sim/scripts/launch_mujoco_fastlio2_live.sh",
        "video",
    ]
    assert (Path(captured["cwd"]) / "lingtu.py").exists()
    assert captured["env"]["LINGTU_PROFILE"] == "explore"
    assert captured["env"]["LINGTU_ENDPOINT"] == "mujoco_live"
    assert captured["env"]["LINGTU_DATA_SOURCE"] == "mujoco_fastlio2_live"
    assert captured["env"]["LINGTU_RUNTIME_CONTRACT"] == "mujoco_fastlio2_live"
    assert captured["check"] is False


def test_external_launcher_uses_runtime_run_spec_env(monkeypatch, capsys):
    import subprocess

    import cli.main as main_mod

    captured = {}

    def _fake_run(cmd, *, cwd, env, check):
        captured["cmd"] = cmd
        captured["cwd"] = cwd
        captured["env"] = env
        captured["check"] = check
        return types.SimpleNamespace(returncode=0)

    monkeypatch.setattr(subprocess, "run", _fake_run)
    monkeypatch.setattr(
        sys,
        "argv",
        ["lingtu.py", "explore", "--endpoint", "mujoco_live", "--record"],
    )

    main_mod.main()

    output = capsys.readouterr().out
    assert (
        "Runtime:  endpoint=mujoco_live data_source=mujoco_fastlio2_live "
        "runtime_contract=mujoco_fastlio2_live module_transport=local "
        "endpoint_transport=local command_sink=mujoco_velocity_adapter "
        "simulation_only=true"
    ) in output
    assert (
        "SLAM:     slam_source=lingtu_fastlio2 localization_source=fastlio2_odometry "
        "mapping_source=fastlio2_map_cloud lidar_extrinsic=mujoco_thunder_v3"
    ) in output
    assert "Frames:   map->odom, odom->body, body->lidar_link" in output
    assert (
        "Topic frames: lidar_scan=lidar_link imu=lidar_link "
        "odometry=odom,map registered_cloud=body "
        "map_cloud=map,odom global_path=map,odom "
        "local_path=map,odom,body cmd_vel=body"
    ) in output
    assert (
        "Flow:     sensors=/points_raw,/imu_raw "
        "localization_map=/nav/odometry,/nav/registered_cloud,/nav/map_cloud "
        "command=mujoco_velocity_adapter"
    ) in output
    assert (
        "Flow stages: endpoint_adapter[endpoint_adapter|native_to_canonical]"
        in output
    )
    assert "global_planning[lingtu_navigation_or_pct|map]" in output
    assert "command_boundary[cmd_vel_mux_to_endpoint_sink|body_twist]" in output
    assert captured["cmd"] == [
        "bash",
        "sim/scripts/launch_mujoco_fastlio2_live.sh",
        "video",
    ]
    assert captured["env"]["LINGTU_PROFILE"] == "explore"
    assert captured["env"]["LINGTU_ENDPOINT"] == "mujoco_live"
    assert captured["env"]["LINGTU_DATA_SOURCE"] == "mujoco_fastlio2_live"
    assert captured["env"]["LINGTU_MODULE_TRANSPORT"] == "local"
    assert captured["env"]["LINGTU_ENDPOINT_TRANSPORT"] == "local"
    assert captured["env"]["LINGTU_RUNTIME_CONTRACT"] == "mujoco_fastlio2_live"
    assert captured["env"]["LINGTU_COMMAND_SINK"] == "mujoco_velocity_adapter"
    assert captured["env"]["LINGTU_SIMULATION_ONLY"] == "1"


def test_external_launcher_overrides_stale_runtime_env(monkeypatch, capsys):
    import subprocess

    import cli.main as main_mod

    monkeypatch.setenv("LINGTU_PROFILE", "nav")
    monkeypatch.setenv("LINGTU_ENDPOINT", "real_s100p")
    monkeypatch.setenv("LINGTU_DATA_SOURCE", "real_s100p")
    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "real_s100p")
    monkeypatch.setenv("LINGTU_COMMAND_SINK", "hardware_driver_after_cmd_vel_mux")
    monkeypatch.setenv("LINGTU_SIMULATION_ONLY", "0")

    captured = {}

    def _fake_run(cmd, *, cwd, env, check):
        captured["cmd"] = cmd
        captured["env"] = env
        captured["check"] = check
        return types.SimpleNamespace(returncode=0)

    monkeypatch.setattr(subprocess, "run", _fake_run)
    monkeypatch.setattr(
        sys,
        "argv",
        ["lingtu.py", "explore", "--endpoint", "mujoco_live", "--record"],
    )

    main_mod.main()

    output = capsys.readouterr().out
    assert "Runtime:  endpoint=mujoco_live data_source=mujoco_fastlio2_live" in output
    assert "command_sink=mujoco_velocity_adapter simulation_only=true" in output
    assert captured["cmd"] == [
        "bash",
        "sim/scripts/launch_mujoco_fastlio2_live.sh",
        "video",
    ]
    assert captured["env"]["LINGTU_PROFILE"] == "explore"
    assert captured["env"]["LINGTU_ENDPOINT"] == "mujoco_live"
    assert captured["env"]["LINGTU_DATA_SOURCE"] == "mujoco_fastlio2_live"
    assert captured["env"]["LINGTU_ENDPOINT_TRANSPORT"] == "local"
    assert captured["env"]["LINGTU_RUNTIME_CONTRACT"] == "mujoco_fastlio2_live"
    assert captured["env"]["LINGTU_COMMAND_SINK"] == "mujoco_velocity_adapter"
    assert captured["env"]["LINGTU_SIMULATION_ONLY"] == "1"


@pytest.mark.sim
def test_tare_product_task_endpoint_record_routes_to_mujoco_live_video(monkeypatch):
    import subprocess

    import cli.main as main_mod

    captured = {}

    def _fake_run(cmd, *, cwd, env, check):
        captured["cmd"] = cmd
        captured["cwd"] = cwd
        captured["env"] = env
        captured["check"] = check
        return types.SimpleNamespace(returncode=0)

    monkeypatch.setattr(subprocess, "run", _fake_run)
    monkeypatch.setattr(
        sys,
        "argv",
        ["lingtu.py", "tare_explore", "--endpoint", "mujoco_live", "--record"],
    )

    main_mod.main()

    assert captured["cmd"] == [
        "bash",
        "sim/scripts/launch_mujoco_fastlio2_live.sh",
        "tare-video",
    ]
    assert (Path(captured["cwd"]) / "lingtu.py").exists()
    assert captured["env"]["LINGTU_PROFILE"] == "tare_explore"
    assert captured["env"]["LINGTU_ENDPOINT"] == "mujoco_live"
    assert captured["env"]["LINGTU_DATA_SOURCE"] == "mujoco_fastlio2_live"
    assert captured["env"]["LINGTU_RUNTIME_CONTRACT"] == "mujoco_fastlio2_live"
    assert captured["check"] is False


def test_endpoint_launcher_accepts_trailing_action_after_options(monkeypatch):
    import subprocess

    import cli.main as main_mod

    captured = {}

    def _fake_run(cmd, *, cwd, env, check):
        captured["cmd"] = cmd
        captured["cwd"] = cwd
        captured["env"] = env
        captured["check"] = check
        return types.SimpleNamespace(returncode=0)

    monkeypatch.setattr(subprocess, "run", _fake_run)
    monkeypatch.setattr(
        sys,
        "argv",
        ["lingtu.py", "explore", "--endpoint", "mujoco_live", "status"],
    )

    main_mod.main()

    assert captured["cmd"] == [
        "bash",
        "sim/scripts/launch_mujoco_fastlio2_live.sh",
        "status",
    ]
    assert captured["env"]["LINGTU_PROFILE"] == "explore"
    assert captured["env"]["LINGTU_ENDPOINT"] == "mujoco_live"


@pytest.mark.sim
def test_mujoco_live_endpoint_accepts_visible_demo_action(monkeypatch):
    import subprocess

    import cli.main as main_mod

    captured = {}

    def _fake_run(cmd, *, cwd, env, check):
        captured["cmd"] = cmd
        captured["cwd"] = cwd
        captured["env"] = env
        captured["check"] = check
        return types.SimpleNamespace(returncode=0)

    monkeypatch.setattr(subprocess, "run", _fake_run)
    monkeypatch.setattr(
        sys,
        "argv",
        ["lingtu.py", "explore", "--endpoint", "mujoco_live", "demo"],
    )

    main_mod.main()

    assert captured["cmd"] == [
        "bash",
        "sim/scripts/launch_mujoco_fastlio2_live.sh",
        "demo",
    ]
    assert captured["env"]["LINGTU_PROFILE"] == "explore"
    assert captured["env"]["LINGTU_ENDPOINT"] == "mujoco_live"


@pytest.mark.sim
def test_switch_plan_prints_sim_to_real_boundary(monkeypatch, capsys):
    import cli.main as main_mod

    monkeypatch.setattr(
        sys,
        "argv",
        ["lingtu.py", "switch-plan", "sim_mujoco_live", "explore"],
    )

    main_mod.main()

    out = capsys.readouterr().out
    assert "Runtime switch plan: PASS" in out
    assert "Current profile: profile=sim_mujoco_live endpoint=mujoco_live" in out
    assert "Target profile: profile=explore endpoint=thunder_field" in out
    assert "Current runtime: endpoint=mujoco_live data_source=mujoco_fastlio2_live" in out
    assert "Target runtime: endpoint=thunder_field data_source=thunder_field" in out
    assert "runtime_contract=thunder_field" in out
    assert "slam_source=lingtu_fastlio_or_external_robot_slam" in out
    assert "mapping_source=slam_map_cloud" in out
    assert "lidar_extrinsic=real_mid360" in out
    assert "Current frame links:" in out
    assert "Target data-flow topics:" in out
    assert "/points_raw,/imu_raw" in out
    assert "/nav/lidar_scan,/nav/imu" in out
    assert "Current data flow:" in out
    assert "Target data flow:" in out
    assert "command_boundary[cmd_vel_mux_to_endpoint_sink|body_twist]" in out
    assert "command_sink" in out
    assert "Current validation: PASS" in out
    assert "Target validation: PASS" in out


def test_switch_plan_json_prints_machine_payload(monkeypatch, capsys):
    import cli.main as main_mod

    monkeypatch.setattr(
        sys,
        "argv",
        ["lingtu.py", "switch-plan", "sim_mujoco_live", "explore", "--json"],
    )

    main_mod.main()

    out = capsys.readouterr().out
    assert '"data_source": "mujoco_fastlio2_live"' in out
    assert '"data_source": "thunder_field"' in out
    assert '"runtime_contract": "thunder_field"' in out
    assert '"slam_source": "lingtu_fastlio_or_external_robot_slam"' in out
    assert '"mapping_source": "slam_map_cloud"' in out
    assert '"lidar_extrinsic_profile": "real_mid360"' in out
    assert '"frame_links": {' in out
    assert '"required_topic_frame_ids": [' in out
    assert '"runtime_data_flow_topics": [' in out
    assert '"resolved_runtime_data_flow": [' in out
    assert '"/points_raw"' in out
    assert '"/nav/lidar_scan"' in out
    assert '"current_validation": {' in out
    assert '"target_validation": {' in out
    assert '"command_sink": "hardware_driver_after_cmd_vel_mux"' in out
    assert '"ok": true' in out


def test_switch_plan_accepts_symmetric_current_endpoint(monkeypatch, capsys):
    import cli.main as main_mod

    monkeypatch.setattr(
        sys,
        "argv",
        [
            "lingtu.py",
            "switch-plan",
            "explore",
            "explore",
            "--current-endpoint",
            "mujoco_live",
            "--endpoint",
            "thunder-field",
            "--json",
        ],
    )

    main_mod.main()

    out = capsys.readouterr().out
    assert '"profile": "explore"' in out
    assert '"endpoint": "mujoco_live"' in out
    assert '"endpoint": "thunder_field"' in out
    assert '"data_source": "mujoco_fastlio2_live"' in out
    assert '"data_source": "thunder_field"' in out
    assert '"simulation_only": true' in out
    assert '"simulation_only": false' in out


def test_switch_plan_exits_when_current_boundary_is_invalid(monkeypatch, capsys):
    import cli.main as main_mod
    import core.runtime_switch as switch_mod
    from core.runtime_switch import RuntimeSwitchValidation

    calls = {"count": 0}

    def _fake_validate(_spec):
        calls["count"] += 1
        if calls["count"] == 1:
            return RuntimeSwitchValidation(False, ("current boundary invalid",))
        return RuntimeSwitchValidation(True, ())

    monkeypatch.setattr(switch_mod, "validate_runtime_switch", _fake_validate)
    monkeypatch.setattr(
        sys,
        "argv",
        ["lingtu.py", "switch-plan", "sim_mujoco_live", "explore"],
    )

    with pytest.raises(SystemExit) as excinfo:
        main_mod.main()

    out = capsys.readouterr().out
    assert excinfo.value.code == 2
    assert "Runtime switch plan: FAIL" in out
    assert "Current validation: FAIL" in out
    assert "  - current boundary invalid" in out
    assert "Target validation: PASS" in out


def test_runtime_spec_prints_single_profile_boundary(monkeypatch, capsys):
    import cli.main as main_mod

    monkeypatch.setattr(
        sys,
        "argv",
        [
            "lingtu.py",
            "runtime-spec",
            "explore",
            "--endpoint",
            "mujoco_live",
            "--json",
        ],
    )

    main_mod.main()

    out = capsys.readouterr().out
    assert '"ok": true' in out
    assert '"validation": {' in out
    assert '"profile": "explore"' in out
    assert '"endpoint": "mujoco_live"' in out
    assert '"data_source": "mujoco_fastlio2_live"' in out
    assert '"runtime_contract": "mujoco_fastlio2_live"' in out
    assert '"command_sink": "mujoco_velocity_adapter"' in out
    assert '"frame_links": {' in out
    assert '"topic_allowed_frame_ids": {' in out
    assert '"/nav/map_cloud": [' in out
    assert '"resolved_runtime_data_flow": [' in out
    assert '"/points_raw"' in out
    assert '"LINGTU_RUNTIME_CONTRACT": "mujoco_fastlio2_live"' in out


def test_runtime_spec_default_prints_operator_summary(monkeypatch, capsys):
    import cli.main as main_mod

    monkeypatch.setattr(
        sys,
        "argv",
        ["lingtu.py", "runtime-spec", "explore", "--endpoint", "mujoco_live"],
    )

    main_mod.main()

    out = capsys.readouterr().out
    assert "Runtime spec: PASS" in out
    assert "Profile: profile=explore endpoint=mujoco_live" in out
    assert (
        "Runtime: endpoint=mujoco_live data_source=mujoco_fastlio2_live "
        "runtime_contract=mujoco_fastlio2_live "
        "module_transport=local endpoint_transport=local "
        "command_sink=mujoco_velocity_adapter "
        "simulation_only=true"
    ) in out
    assert "SLAM: slam_source=lingtu_fastlio2" in out
    assert "Frames: map=map odom=odom body=body lidar=lidar_link" in out
    assert "Topic frames:" in out
    assert "  map_cloud=map,odom" in out
    assert "Data flow:" in out
    assert "  endpoint_adapter[endpoint_adapter|native_to_canonical]" in out
    assert "Runtime env:" in out
    assert "  LINGTU_MODULE_TRANSPORT=local" in out
    assert "  LINGTU_ENDPOINT_TRANSPORT=local" in out
    assert "  LINGTU_RUNTIME_CONTRACT=mujoco_fastlio2_live" in out


def test_runtime_spec_accepts_thunder_product_alias(monkeypatch, capsys):
    import cli.main as main_mod

    monkeypatch.setattr(
        sys,
        "argv",
        ["lingtu.py", "runtime-spec", "thunder-explore", "--endpoint", "mujoco_live"],
    )

    main_mod.main()

    out = capsys.readouterr().out
    assert "Runtime spec: PASS" in out
    assert "Profile: profile=explore endpoint=mujoco_live" in out


def test_runtime_spec_exits_when_boundary_is_invalid(monkeypatch, capsys):
    import cli.main as main_mod
    import core.runtime_switch as switch_mod
    from core.runtime_switch import RuntimeSwitchValidation

    monkeypatch.setattr(
        switch_mod,
        "validate_runtime_switch",
        lambda _spec: RuntimeSwitchValidation(False, ("runtime boundary invalid",)),
    )
    monkeypatch.setattr(
        sys,
        "argv",
        ["lingtu.py", "runtime-spec", "explore", "--endpoint", "mujoco_live"],
    )

    with pytest.raises(SystemExit) as excinfo:
        main_mod.main()

    out = capsys.readouterr().out
    assert excinfo.value.code == 2
    assert "Runtime spec: FAIL" in out
    assert "Validation blockers:" in out
    assert "  runtime boundary invalid" in out


def test_runtime_contract_prints_canonical_manifest(monkeypatch, capsys):
    import cli.main as main_mod

    monkeypatch.setattr(
        sys,
        "argv",
        ["lingtu.py", "runtime-contract", "--json"],
    )

    main_mod.main()

    payload = json.loads(capsys.readouterr().out)
    assert payload["schema_version"] == "lingtu.runtime_interface.v1"
    assert payload["frames"]["map"] == "map"
    assert payload["frame_links"]["map_to_odom"] == {
        "parent": "map",
        "child": "odom",
        "required": True,
    }
    assert payload["topics"]["cmd_vel"] == "/nav/cmd_vel"
    assert payload["runtime_data_flow_topics"]["thunder_field"] == [
        "/nav/lidar_scan",
        "/nav/imu",
        "/nav/odometry",
        "/nav/registered_cloud",
        "/nav/map_cloud",
        "/nav/localization_health",
        "/nav/localization_quality",
        "/nav/exploration_grid",
        "/nav/terrain_map_ext",
        "/exploration/way_point",
        "/nav/goal_pose",
        "/nav/traversable_frontiers",
        "/nav/frontier_candidate",
        "/nav/global_path",
        "/nav/way_point",
        "/nav/local_path",
        "/nav/cmd_vel",
        "/nav/added_obstacles",
        "/nav/check_obstacle",
        "/nav/planner_status",
    ]


def test_runtime_contract_default_prints_operator_summary(monkeypatch, capsys):
    import cli.main as main_mod

    monkeypatch.setattr(
        sys,
        "argv",
        ["lingtu.py", "runtime-contract"],
    )

    main_mod.main()

    out = capsys.readouterr().out
    assert "Runtime contract: lingtu.runtime_interface.v1" in out
    assert "Frames: map=map odom=odom body=body lidar=lidar_link" in out
    assert "Real topic frames:" in out
    assert "  odometry=odom,map" in out
    assert "  map_cloud=map" in out
    assert "  local_path=map,odom,body" in out
    assert "Real data flow:" in out
    assert "  endpoint_adapter" in out
    assert "Data sources:" in out
    assert (
        "  thunder_field[hardware] source=/nav/lidar_scan,/nav/imu "
        "normalized=/nav/lidar_scan,/nav/imu"
    ) in out
    assert "Profile bindings:" in out
    assert "  nav->thunder_field mode=real_robot_saved_map_navigation" in out
    assert "Artifact formats:" in out
    assert "  tomogram path=tomogram.pickle type=pct_tomogram frame_role=map" in out
    assert "Adapter aliases:" in out
    assert "  fastlio2 /cloud_registered->/nav/registered_cloud" in out
    assert "Adapter relays:" in out
    assert "  cmu_unity /state_estimation->/nav/odometry" in out
    assert "/nav/cmd_vel->/cmd_vel(geometry_msgs/msg/TwistStamped)" in out
    assert "Algorithm interfaces:" in out
    assert "  fastlio_mapping[slam|" in out


def test_runtime_contract_writes_json_out(monkeypatch, tmp_path, capsys):
    import cli.main as main_mod

    out_path = tmp_path / "runtime_contract.json"
    monkeypatch.setattr(
        sys,
        "argv",
        ["lingtu.py", "runtime-contract", "--json-out", str(out_path)],
    )

    main_mod.main()

    assert capsys.readouterr().out == ""
    payload = json.loads(out_path.read_text(encoding="utf-8"))
    assert payload["schema_version"] == "lingtu.runtime_interface.v1"
    assert payload["real_runtime_required_topic_frame_ids"] == [
        "/nav/lidar_scan",
        "/nav/imu",
        "/nav/odometry",
        "/nav/registered_cloud",
        "/nav/map_cloud",
        "/nav/global_path",
        "/nav/local_path",
        "/nav/cmd_vel",
    ]
    assert payload["real_runtime_required_endpoint_input_topics"] == [
        "/nav/lidar_scan",
        "/nav/imu",
    ]


def test_runtime_audit_prints_contract_gate(monkeypatch, capsys):
    import cli.main as main_mod

    monkeypatch.setattr(
        sys,
        "argv",
        ["lingtu.py", "runtime-audit", "--json"],
    )

    main_mod.main()

    out = capsys.readouterr().out
    assert '"schema_version": "lingtu.runtime_contract_audit.v1"' in out
    assert '"ok": true' in out
    assert '"validation_gate": {' in out
    assert '"acceptance_step": 1' in out
    assert '"operator_summary_sections": [' in out
    assert '"yaml_manifest": {' in out
    assert '"profile_runtime_specs": {' in out
    assert '"real_runtime_collector": {' in out
    assert '"source_frame_contracts": {' in out
    assert '"source_topic_contracts": {' in out
    assert '"src/drivers/sim/mujoco_sensor_bridge.py"' in out
    assert '"sim/scripts/mujoco_fastlio2_live_gate.py"' in out
    assert '"sim/scripts/saved_map_relocalize_runtime_gate.py"' in out
    assert '"scripts/monitor/feishu_monitor_bot.py"' in out
    assert '"required_thunder_field_topics": [' in out


def test_runtime_audit_default_prints_operator_summary(monkeypatch, capsys):
    import cli.main as main_mod

    monkeypatch.setattr(
        sys,
        "argv",
        ["lingtu.py", "runtime-audit"],
    )

    main_mod.main()

    out = capsys.readouterr().out
    assert "Runtime audit: PASS" in out
    assert "Schema: lingtu.runtime_contract_audit.v1" in out
    assert "Validation gate:" in out
    assert (
        "  step=1 "
        "required_when=before_any_runtime_contract_or_field_readiness_claim"
    ) in out
    assert "Checks:" in out
    assert "  yaml_manifest ok=true blockers=0" in out
    assert "  runtime_validation_gates ok=true blockers=0" in out
    assert "Validation gate sequence:" in out
    assert (
        "  step=1 runtime_audit "
        "required_when=before_any_runtime_contract_or_field_readiness_claim"
    ) in out
    assert (
        "  step=2 saved_map_artifact_gate "
        "required_when=saved_map_tomogram_occupancy_or_pct_artifact_is_used"
    ) in out
    assert (
        "  step=3 real_runtime_evidence "
        "required_when=before_claiming_thunder_field_runtime_or_field_navigation "
        "prior=runtime_audit"
    ) in out
    assert "Validation commands:" in out
    assert "  runtime_audit=python lingtu.py runtime-audit" in out
    assert '"schema_version":' not in out


def test_runtime_audit_writes_json_out(monkeypatch, tmp_path, capsys):
    import cli.main as main_mod

    out_path = tmp_path / "runtime_contract_audit.json"
    monkeypatch.setattr(
        sys,
        "argv",
        ["lingtu.py", "runtime-audit", "--json-out", str(out_path)],
    )

    main_mod.main()

    assert capsys.readouterr().out == ""
    payload = json.loads(out_path.read_text(encoding="utf-8"))
    assert payload["schema_version"] == "lingtu.runtime_contract_audit.v1"
    assert payload["ok"] is True
    assert payload["validation_gate"]["acceptance_step"] == 1
    assert payload["checks"]["source_frame_contracts"]["ok"] is True
    assert payload["checks"]["source_frame_contracts"]["matches"] == []
    assert (
        "sim/scripts/mujoco_fastlio2_live_gate.py"
        in payload["checks"]["source_frame_contracts"]["checked_files"]
    )
    assert (
        "src/gateway/schemas.py"
        in payload["checks"]["source_frame_contracts"]["checked_files"]
    )
    assert (
        "src/core/msgs/geometry.py"
        in payload["checks"]["source_frame_contracts"]["checked_files"]
    )
    assert (
        "src/core/msgs/nav.py"
        in payload["checks"]["source_frame_contracts"]["checked_files"]
    )
    assert (
        "src/drivers/real/lidar/lidar_module.py"
        in payload["checks"]["source_frame_contracts"]["checked_files"]
    )
    assert (
        "src/drivers/real/thunder/connection.py"
        in payload["checks"]["source_frame_contracts"]["checked_files"]
    )
    assert (
        "src/drivers/real/thunder/han_dog_module.py"
        in payload["checks"]["source_frame_contracts"]["checked_files"]
    )
    assert (
        "src/semantic/planner/semantic_planner_module.py"
        in payload["checks"]["source_frame_contracts"]["checked_files"]
    )
    assert (
        "src/semantic/perception/perception_module.py"
        in payload["checks"]["source_frame_contracts"]["checked_files"]
    )
    assert (
        "sim/scripts/saved_map_relocalize_runtime_gate.py"
        in payload["checks"]["source_topic_contracts"]["checked_files"]
    )
    assert payload["checks"]["source_topic_contracts"]["ok"] is True
    assert payload["checks"]["source_topic_contracts"]["matches"] == []
    assert (
        "scripts/monitor/feishu_config_template.py"
        in payload["checks"]["source_topic_contracts"]["checked_files"]
    )


def test_runtime_audit_source_frame_contracts_reject_direct_frame_constants(
    monkeypatch, tmp_path
):
    import cli.runtime_audit as audit_mod

    source_root = tmp_path / "src" / "slam"
    source_root.mkdir(parents=True)
    (source_root / "good.py").write_text(
        "FRAME_ID = topic_default_frame_id(TOPICS.odometry)\n"
        "CONFIG = {'planning_frame_id': map_frame_id()}\n"
        "class Good:\n"
        "    frame_id: str = map_frame_id()\n"
        "frame_id = getattr(body, 'frame_id', map_frame_id())\n",
        encoding="utf-8",
    )
    bad_source = source_root / "bad_sim_driver.py"
    bad_source.write_text(
        'frame_id = FRAMES.odom\n'
        'CONFIG = {"planning_frame_id": "map"}\n'
        'class Bad:\n'
        '    frame_id: str = "map"\n'
        'frame_id = getattr(body, "frame_id", "map")\n'
        'PAYLOAD = {"frame_id": parsed.get("frame_id") or "map"}\n'
        'child_frame_id = "body"\n'
        '_odom_frame_id = "map"\n',
        encoding="utf-8",
    )
    monkeypatch.setattr(audit_mod, "REPO_ROOT", tmp_path)
    monkeypatch.setattr(
        audit_mod,
        "SOURCE_FRAME_CONTRACT_ROOTS",
        ("src/slam",),
    )

    result = audit_mod._check_source_frame_contracts()

    assert result["ok"] is False
    assert result["checked_files"] == [
        "src/slam/bad_sim_driver.py",
        "src/slam/good.py",
    ]
    assert result["matches"] == [
        {
            "file": "src/slam/bad_sim_driver.py",
            "line": 1,
            "pattern": "direct_FRAMES_runtime_frames",
            "text": "frame_id = FRAMES.odom",
        },
        {
            "file": "src/slam/bad_sim_driver.py",
            "line": 2,
            "pattern": "hardcoded_frame_config_literal",
            "text": 'CONFIG = {"planning_frame_id": "map"}',
        },
        {
            "file": "src/slam/bad_sim_driver.py",
            "line": 4,
            "pattern": "hardcoded_frame_field_default",
            "text": 'frame_id: str = "map"',
        },
        {
            "file": "src/slam/bad_sim_driver.py",
            "line": 5,
            "pattern": "hardcoded_frame_default_argument",
            "text": 'frame_id = getattr(body, "frame_id", "map")',
        },
        {
            "file": "src/slam/bad_sim_driver.py",
            "line": 6,
            "pattern": "hardcoded_frame_or_default",
            "text": 'PAYLOAD = {"frame_id": parsed.get("frame_id") or "map"}',
        },
        {
            "file": "src/slam/bad_sim_driver.py",
            "line": 7,
            "pattern": "hardcoded_frame_id_assignment",
            "text": 'child_frame_id = "body"',
        },
        {
            "file": "src/slam/bad_sim_driver.py",
            "line": 8,
            "pattern": "hardcoded_frame_config_literal",
            "text": '_odom_frame_id = "map"',
        },
    ]
    assert (
        "source frame contract violation src/slam/bad_sim_driver.py:1 "
        "direct_FRAMES_runtime_frames"
    ) in result["blockers"]


def test_runtime_audit_source_topic_contracts_reject_direct_runtime_topics(
    monkeypatch, tmp_path
):
    import cli.runtime_audit as audit_mod

    source_root = tmp_path / "src" / "nav"
    source_root.mkdir(parents=True)
    (source_root / "good.py").write_text(
        'TOPIC = TOPICS.goal_pose\nPATH = "src/nav/navigation_module.py"\n',
        encoding="utf-8",
    )
    bad_source = source_root / "bad_nav_driver.py"
    bad_source.write_text(
        'ERROR = f"topic /nav/goal_pose missing"\n',
        encoding="utf-8",
    )
    monkeypatch.setattr(audit_mod, "REPO_ROOT", tmp_path)
    monkeypatch.setattr(
        audit_mod,
        "SOURCE_TOPIC_CONTRACT_ROOTS",
        ("src/nav",),
    )

    result = audit_mod._check_source_topic_contracts()

    assert result["ok"] is False
    assert result["checked_files"] == [
        "src/nav/bad_nav_driver.py",
        "src/nav/good.py",
    ]
    assert result["matches"] == [
        {
            "file": "src/nav/bad_nav_driver.py",
            "line": 1,
            "pattern": "hardcoded_canonical_runtime_topic",
            "text": 'ERROR = f"topic /nav/goal_pose missing"',
        }
    ]
    assert (
        "source topic contract violation src/nav/bad_nav_driver.py:1 "
        "hardcoded_canonical_runtime_topic"
    ) in result["blockers"]


def test_saved_map_artifact_gate_cli_invokes_script(monkeypatch, tmp_path):
    import subprocess

    import cli.main as main_mod

    captured = {}

    def _fake_run(cmd, *, cwd, check):
        captured["cmd"] = cmd
        captured["cwd"] = cwd
        captured["check"] = check
        return types.SimpleNamespace(returncode=0)

    map_dir = tmp_path / "map"
    out_path = tmp_path / "artifact_gate.json"
    monkeypatch.setattr(subprocess, "run", _fake_run)
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "lingtu.py",
            "saved-map-artifact-gate",
            str(map_dir),
            "--require-tomogram",
            "--require-occupancy",
            "--expected-data-source",
            "thunder_field",
            "--expected-source-profile",
            "nav",
            "--expected-frame-id",
            "map",
            "--json-out",
            str(out_path),
        ],
    )

    main_mod.main()

    cmd = captured["cmd"]
    assert cmd[0] == sys.executable
    assert Path(cmd[1]).name == "saved_map_artifact_gate.py"
    assert cmd[2] == str(map_dir)
    assert "--require-tomogram" in cmd
    assert "--require-occupancy" in cmd
    assert cmd[cmd.index("--expected-data-source") + 1] == "thunder_field"
    assert cmd[cmd.index("--expected-source-profile") + 1] == "nav"
    assert cmd[cmd.index("--expected-frame-id") + 1] == "map"
    assert cmd[cmd.index("--json-out") + 1] == str(out_path)
    assert (Path(captured["cwd"]) / "lingtu.py").exists()
    assert captured["check"] is False
    assert "--json" not in cmd


@pytest.mark.sim
def test_field_check_cli_defaults_to_simulation_mode_and_writes_json(
    monkeypatch, tmp_path, capsys
):
    import cli.main as main_mod
    import core.product_field_check as field_check_mod

    captured = {}
    out_path = tmp_path / "field_check.json"
    map_dir = tmp_path / "map"

    def _fake_collect(**kwargs):
        captured.update(kwargs)
        return {
            "schema_version": "lingtu.product_field_check.v1",
            "ok": True,
            "summary": "PASS",
            "mode": kwargs["mode"],
            "map": {},
            "runtime": {},
            "navigation": {},
            "evidence": {},
            "blockers": [],
            "advisories": [],
        }

    monkeypatch.setattr(field_check_mod, "collect_product_field_check", _fake_collect)
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "lingtu.py",
            "field-check",
            str(map_dir),
            "--gateway-url",
            "http://robot.local:5050",
            "--gateway-timeout-sec",
            "3.5",
            "--require-tomogram",
            "--require-occupancy",
            "--expected-data-source",
            "thunder_field",
            "--expected-source-profile",
            "nav",
            "--expected-frame-id",
            "map",
            "--json-out",
            str(out_path),
        ],
    )

    main_mod.main()

    assert capsys.readouterr().out == ""
    assert captured == {
        "gateway_url": "http://robot.local:5050",
        "timeout_sec": 3.5,
        "mode": "simulation",
        "map_dir": str(map_dir),
        "require_tomogram": True,
        "require_occupancy": True,
        "expected_data_source": "thunder_field",
        "expected_source_profile": "nav",
        "expected_frame_id": "map",
    }
    payload = json.loads(out_path.read_text(encoding="utf-8"))
    assert payload["schema_version"] == "lingtu.product_field_check.v1"
    assert payload["mode"] == "simulation"


def test_dataflow_cli_fetches_one_topic_without_ros2(monkeypatch, capsys):
    import cli.main as main_mod

    captured = {}

    def _fake_get(gateway_url, path, *, timeout_sec, query=None):
        captured.update(
            {
                "gateway_url": gateway_url,
                "path": path,
                "timeout_sec": timeout_sec,
                "query": query,
            }
        )
        return {
            "schema_version": 1,
            "ok": True,
            "selector": "odometry",
            "topic": {"topic": "/nav/odometry"},
            "inspection": {
                "live": True,
                "observation_level": "fresh_module_sample",
                "payload_interfaces": [
                    {"transport": "gateway_rest", "path": "/api/v1/state"},
                    {"transport": "gateway_sse", "path": "/api/v1/events"},
                ],
                "communicate": False,
                "write_interfaces": [],
                "ros2_topic_required": False,
                "arbitrary_publish_supported": False,
            },
        }

    monkeypatch.setattr(main_mod, "_gateway_get_json", _fake_get)
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "lingtu.py",
            "dataflow",
            "odometry",
            "--gateway-url",
            "http://robot.local:5050",
            "--gateway-timeout-sec",
            "3.5",
        ],
    )

    main_mod.main()

    out = capsys.readouterr().out
    assert captured == {
        "gateway_url": "http://robot.local:5050",
        "path": "/api/v1/runtime/dataflow/topic",
        "timeout_sec": 3.5,
        "query": {"topic": "odometry"},
    }
    assert "Runtime dataflow topic: PASS" in out
    assert "topic=/nav/odometry selector=odometry" in out
    assert "live=true observation=fresh_module_sample" in out
    assert "gateway_rest:/api/v1/state" in out
    assert "gateway_sse:/api/v1/events" in out
    assert "communication=read_only" in out
    assert "endpoint_topic_required=false" in out
    assert "arbitrary_publish_supported=false" in out


def test_dataflow_cli_summary_json(monkeypatch, capsys):
    import cli.main as main_mod

    def _fake_get(gateway_url, path, *, timeout_sec, query=None):
        return {
            "schema_version": 1,
            "runtime_contract": "thunder_field",
            "transport_layers": {
                "module_port_bus": {"primary": True},
                "ros2_adapter": {"primary": False},
            },
            "topics": [
                {
                    "topic": "/nav/odometry",
                    "inspection": {
                        "live": True,
                        "ros2_topic_required": False,
                        "arbitrary_publish_supported": False,
                    },
                }
            ],
            "control_boundary": {"arbitrary_publish_supported": False},
        }

    monkeypatch.setattr(main_mod, "_gateway_get_json", _fake_get)
    monkeypatch.setattr(sys, "argv", ["lingtu.py", "dataflow", "--json"])

    main_mod.main()

    payload = json.loads(capsys.readouterr().out)
    assert payload["runtime_contract"] == "thunder_field"
    assert payload["transport_layers"]["module_port_bus"]["primary"] is True


def test_dataflow_cli_summary_includes_stage_and_command_closure(
    monkeypatch,
    capsys,
):
    import cli.main as main_mod

    def _fake_get(gateway_url, path, *, timeout_sec, query=None):
        return {
            "schema_version": 1,
            "runtime_contract": "thunder_field",
            "transport_layers": {
                "module_port_bus": {"primary": True},
                "ros2_adapter": {"primary": False},
            },
            "topics": [
                {
                    "topic": "/nav/odometry",
                    "inspection": {"live": True},
                    "communication": {"allowed": False},
                },
                {
                    "topic": "/nav/cmd_vel",
                    "inspection": {"live": True},
                    "communication": {"allowed": True},
                },
            ],
            "stage_evidence": [
                {"name": "global_planning", "live": True, "observable": True},
                {
                    "name": "local_planning_and_following",
                    "live": False,
                    "observable": False,
                    "status": "missing",
                },
            ],
            "control_boundary": {
                "arbitrary_publish_supported": False,
                "command_interfaces": [
                    {"path": "/api/v1/goal"},
                    {"path": "/api/v1/cmd_vel"},
                    {"path": "/api/v1/stop"},
                ],
            },
        }

    monkeypatch.setattr(main_mod, "_gateway_get_json", _fake_get)
    monkeypatch.setattr(sys, "argv", ["lingtu.py", "dataflow"])

    main_mod.main()

    out = capsys.readouterr().out
    assert "Runtime dataflow: PASS" in out
    assert "topics=2 live_topics=2" in out
    assert "stages=2 live_stages=1 missing_stages=1" in out
    assert "commandable_topics=1 command_interfaces=3" in out
    assert "endpoint_adapter.primary=false" in out


def test_dataflow_cli_gateway_failure_reports_fail(monkeypatch, capsys):
    import cli.main as main_mod

    def _fake_get(gateway_url, path, *, timeout_sec, query=None):
        return {
            "ok": False,
            "error": "gateway_request_failed",
            "message": "timed out",
            "url": "http://robot.local:5050/api/v1/runtime/dataflow",
        }

    monkeypatch.setattr(main_mod, "_gateway_get_json", _fake_get)
    monkeypatch.setattr(sys, "argv", ["lingtu.py", "dataflow"])

    with pytest.raises(SystemExit) as exc:
        main_mod.main()

    assert exc.value.code == 2
    out = capsys.readouterr().out
    assert "Runtime dataflow: FAIL" in out
    assert "error=gateway_request_failed" in out
    assert "message=timed out" in out
    assert "url=http://robot.local:5050/api/v1/runtime/dataflow" in out


def test_dataflow_cli_unknown_topic_exits_nonzero(monkeypatch, capsys):
    import cli.main as main_mod

    def _fake_get(gateway_url, path, *, timeout_sec, query=None):
        return {
            "schema_version": 1,
            "ok": False,
            "selector": query["topic"] if query else None,
            "error": "runtime_topic_not_found",
            "topic": None,
            "inspection": {
                "observable": False,
                "communicate": False,
                "ros2_topic_required": False,
                "arbitrary_publish_supported": False,
            },
            "available_topics": ["/nav/odometry"],
        }

    monkeypatch.setattr(main_mod, "_gateway_get_json", _fake_get)
    monkeypatch.setattr(sys, "argv", ["lingtu.py", "dataflow", "missing_stream"])

    with pytest.raises(SystemExit) as exc:
        main_mod.main()

    assert exc.value.code == 2
    out = capsys.readouterr().out
    assert "Runtime dataflow topic: FAIL" in out
    assert "error=runtime_topic_not_found" in out
    assert "endpoint_topic_required=false" in out


def test_cli_list_defaults_to_product_profiles_and_all_expands(
    monkeypatch,
    capsys,
):
    import cli.main as main_mod

    monkeypatch.setattr(sys, "argv", ["lingtu.py", "--list"])
    main_mod.main()
    out = capsys.readouterr().out

    assert "Available product profiles" in out
    assert "lite" in out
    assert "thunder-lite" in out
    assert "map" in out
    assert "nav" in out
    assert "thunder-nav" in out
    assert "explore" in out
    assert "sim_mujoco_live" not in out
    assert "Use --list --all" in out

    monkeypatch.setattr(sys, "argv", ["lingtu.py", "--list", "--all"])
    main_mod.main()
    out = capsys.readouterr().out

    assert "Available profiles" in out
    assert "sim_mujoco_live" in out
    assert "super_lio" in out


def test_saved_map_artifact_gate_cli_forwards_json_flag(monkeypatch, tmp_path):
    import subprocess

    import cli.main as main_mod

    captured = {}

    def _fake_run(cmd, *, cwd, check):
        captured["cmd"] = cmd
        captured["cwd"] = cwd
        captured["check"] = check
        return types.SimpleNamespace(returncode=0)

    map_dir = tmp_path / "map"
    monkeypatch.setattr(subprocess, "run", _fake_run)
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "lingtu.py",
            "saved-map-artifact-gate",
            str(map_dir),
            "--json",
        ],
    )

    main_mod.main()

    cmd = captured["cmd"]
    assert Path(cmd[1]).name == "saved_map_artifact_gate.py"
    assert cmd[2] == str(map_dir)
    assert "--json" in cmd


def test_real_runtime_evidence_cli_runs_read_only_collector(
    monkeypatch, tmp_path, capsys
):
    import subprocess

    import cli.main as main_mod

    captured = {}

    def _fake_run(cmd, *, cwd, check):
        captured["cmd"] = cmd
        captured["cwd"] = cwd
        captured["check"] = check
        return types.SimpleNamespace(returncode=0)

    out_path = tmp_path / "thunder_field_runtime" / "report.json"
    monkeypatch.setattr(subprocess, "run", _fake_run)
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "lingtu.py",
            "real-runtime-evidence",
            "--duration-sec",
            "3.5",
            "--min-motion-m",
            "0.2",
            "--min-cmd-vel-norm",
            "0.07",
            "--expected-command-subscriber",
            "s100p_driver",
            "--json-out",
            str(out_path),
        ],
    )

    main_mod.main()

    out = capsys.readouterr().out
    assert f"Real runtime evidence report: {out_path}" in out
    cmd = captured["cmd"]
    assert cmd[0] == sys.executable
    assert Path(cmd[1]).name == "real_runtime_evidence_collect.py"
    assert cmd[cmd.index("--duration-sec") + 1] == "3.5"
    assert cmd[cmd.index("--min-motion-m") + 1] == "0.2"
    assert cmd[cmd.index("--min-cmd-vel-norm") + 1] == "0.07"
    assert cmd[cmd.index("--expected-contract") + 1] == "thunder_field"
    assert cmd[cmd.index("--expected-command-subscriber") + 1] == "s100p_driver"
    assert cmd[cmd.index("--json-out") + 1] == str(out_path)
    assert "--no-validate" not in cmd
    assert "--json" not in cmd
    assert (Path(captured["cwd"]) / "lingtu.py").exists()
    assert captured["check"] is False


def test_real_runtime_evidence_cli_propagates_gate_failure(monkeypatch, capsys):
    import subprocess

    import cli.main as main_mod

    captured = {}

    def _fake_run(cmd, *, cwd, check):
        captured["cmd"] = cmd
        return types.SimpleNamespace(returncode=2)

    monkeypatch.setattr(subprocess, "run", _fake_run)
    monkeypatch.setattr(
        sys,
        "argv",
        ["lingtu.py", "real-runtime-evidence", "--no-validate"],
    )

    with pytest.raises(SystemExit) as excinfo:
        main_mod.main()

    out = capsys.readouterr().out
    assert excinfo.value.code == 2
    assert "Real runtime evidence report: artifacts" in out
    assert "--no-validate" in captured["cmd"]


def test_real_runtime_evidence_cli_prints_validation_blockers(
    monkeypatch, tmp_path, capsys
):
    import subprocess

    import cli.main as main_mod

    def _fake_run(cmd, *, cwd, check):
        json_out = Path(cmd[cmd.index("--json-out") + 1])
        json_out.parent.mkdir(parents=True, exist_ok=True)
        json_out.write_text(
            json.dumps(
                {
                    "real_robot_motion": False,
                    "cmd_vel_sent_to_hardware": False,
                    "motion": {"odom_delta_m": 0.0, "min_motion_m": 0.2},
                    "outputs": {
                        "global_path_count": 0,
                        "local_path_count": 0,
                        "nav_cmd_vel_nonzero": 0,
                    },
                    "hardware_boundary": {
                        "command_sink": "hardware_driver_after_cmd_vel_mux",
                    },
                    "runtime_contract": {"name": "thunder_field", "ok": False},
                    "runtime_evidence": {
                        "ok": False,
                        "validation_gate": {
                            "acceptance_step": 3,
                            "required_when": (
                                "before_claiming_thunder_field_runtime_or_field_navigation"
                            ),
                            "requires_prior_gates": ["runtime_audit"],
                            "conditional_prior_gates": [
                                "saved_map_artifact_gate when saved map, tomogram, occupancy, or PCT artifact is used"
                            ],
                            "proves": [
                                "observed_thunder_field_runtime_contract",
                                "observed_resolved_runtime_data_flow",
                            ],
                            "operator_summary_sections": [
                                "Blockers",
                                "Topic frame evidence",
                                "Frame link evidence",
                                "Data-flow evidence",
                            ],
                        },
                        "blockers": [
                            "real robot motion evidence missing",
                            "cmd_vel did not reach hardware boundary",
                        ],
                        "checked_runtime_topics": [
                            "/nav/odometry",
                            "/nav/cmd_vel",
                        ],
                        "checked_frame_links": ["map_to_odom"],
                        "checked_data_flow_stages": ["command_boundary"],
                        "checked_required_topic_frame_report": {
                            "/nav/odometry": {
                                "default_frame_id": "odom",
                                "observed_frame_id": None,
                                "allowed_frame_ids": ["odom", "map"],
                                "ok": False,
                            },
                            "/nav/cmd_vel": {
                                "default_frame_id": "body",
                                "observed_frame_id": "body",
                                "allowed_frame_ids": ["body"],
                                "ok": True,
                            },
                        },
                        "checked_frame_link_evidence": {
                            "map_to_odom": {
                                "expected_parent": "map",
                                "expected_child": "odom",
                                "observed_parent": None,
                                "observed_child": None,
                                "samples": 0,
                                "static": False,
                                "published": False,
                                "error": "tf missing",
                                "ok": False,
                            }
                        },
                        "checked_runtime_data_flow_evidence": {
                            "command_boundary": {
                                "ok": False,
                                "required": True,
                                "observed_inputs": ["/nav/cmd_vel"],
                                "observed_outputs": [],
                                "missing_inputs": [],
                                "missing_outputs": [
                                    "hardware_driver_after_cmd_vel_mux"
                                ],
                                "missing_signals": ["hardware_command_route"],
                                "owner": "cmd_vel_mux_to_endpoint_sink",
                                "frame_role": "body_twist",
                                "reason": "hardware boundary missing",
                            }
                        },
                    },
                }
            )
            + "\n",
            encoding="utf-8",
        )
        return types.SimpleNamespace(returncode=2)

    out_path = tmp_path / "thunder_field_runtime" / "report.json"
    monkeypatch.setattr(subprocess, "run", _fake_run)
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "lingtu.py",
            "real-runtime-evidence",
            "--json-out",
            str(out_path),
        ],
    )

    with pytest.raises(SystemExit) as excinfo:
        main_mod.main()

    out = capsys.readouterr().out
    assert excinfo.value.code == 2
    assert f"Real runtime evidence report: {out_path}" in out
    assert "Real runtime evidence: FAIL" in out
    assert "Runtime contract: name=thunder_field ok=false" in out
    assert "Validation gate:" in out
    assert (
        "  step=3 "
        "required_when=before_claiming_thunder_field_runtime_or_field_navigation "
        "prior=runtime_audit"
    ) in out
    assert "  real robot motion evidence missing" in out
    assert "  cmd_vel did not reach hardware boundary" in out
    assert "  /nav/odometry" in out
    assert "  map_to_odom" in out
    assert "Topic frame evidence:" in out
    assert "  /nav/odometry default=odom observed=missing" in out
    assert "Frame link evidence:" in out
    assert "  map_to_odom expected=map->odom observed=missing" in out
    assert "Data-flow evidence:" in out
    assert "  command_boundary[cmd_vel_mux_to_endpoint_sink|body_twist] ok=false" in out


@pytest.mark.sim
def test_mujoco_live_endpoint_accepts_pct_moving_obstacle_action(monkeypatch):
    import subprocess

    import cli.main as main_mod

    captured = {}

    def _fake_run(cmd, *, cwd, env, check):
        captured["cmd"] = cmd
        captured["cwd"] = cwd
        captured["env"] = env
        captured["check"] = check
        return types.SimpleNamespace(returncode=0)

    monkeypatch.setattr(subprocess, "run", _fake_run)
    monkeypatch.setattr(
        sys,
        "argv",
        ["lingtu.py", "sim_mujoco_live", "pct-moving-obstacle"],
    )

    main_mod.main()

    assert captured["cmd"] == [
        "bash",
        "sim/scripts/launch_mujoco_fastlio2_live.sh",
        "pct-moving-obstacle",
    ]
    assert captured["env"]["LINGTU_PROFILE"] == "sim_mujoco_live"
    assert captured["env"]["LINGTU_RUNTIME_CONTRACT"] == "mujoco_fastlio2_live"


@pytest.mark.sim
def test_mujoco_live_endpoint_accepts_inspection_video_action(monkeypatch):
    import subprocess

    import cli.main as main_mod

    captured = {}

    def _fake_run(cmd, *, cwd, env, check):
        captured["cmd"] = cmd
        captured["cwd"] = cwd
        captured["env"] = env
        captured["check"] = check
        return types.SimpleNamespace(returncode=0)

    monkeypatch.setattr(subprocess, "run", _fake_run)
    monkeypatch.setattr(
        sys,
        "argv",
        ["lingtu.py", "sim_mujoco_live", "inspection-moving-obstacle-video"],
    )

    main_mod.main()

    assert captured["cmd"] == [
        "bash",
        "sim/scripts/launch_mujoco_fastlio2_live.sh",
        "inspection-moving-obstacle-video",
    ]
    assert captured["env"]["LINGTU_PROFILE"] == "sim_mujoco_live"
    assert captured["env"]["LINGTU_RUNTIME_CONTRACT"] == "mujoco_fastlio2_live"


@pytest.mark.sim
def test_tare_product_task_endpoint_routes_to_cmu_unity_launcher(monkeypatch):
    import subprocess

    import cli.main as main_mod

    captured = {}

    def _fake_run(cmd, *, cwd, env, check):
        captured["cmd"] = cmd
        captured["cwd"] = cwd
        captured["env"] = env
        captured["check"] = check
        return types.SimpleNamespace(returncode=0)

    monkeypatch.setattr(subprocess, "run", _fake_run)
    monkeypatch.setattr(
        sys,
        "argv",
        ["lingtu.py", "tare_explore", "--endpoint", "cmu_unity"],
    )

    main_mod.main()

    assert captured["cmd"] == [
        "bash",
        "sim/scripts/launch_cmu_unity_lingtu_runtime.sh",
        "gate",
    ]
    assert (Path(captured["cwd"]) / "lingtu.py").exists()
    assert captured["env"]["LINGTU_PROFILE"] == "tare_explore"
    assert captured["env"]["LINGTU_ENDPOINT"] == "cmu_unity"
    assert captured["env"]["LINGTU_DATA_SOURCE"] == "cmu_unity_external"
    assert captured["env"]["LINGTU_RUNTIME_CONTRACT"] == "cmu_unity_external"
    assert captured["check"] is False


@pytest.mark.sim
def test_tare_product_task_endpoint_routes_to_mujoco_live_launcher(monkeypatch):
    import subprocess

    import cli.main as main_mod

    captured = {}

    def _fake_run(cmd, *, cwd, env, check):
        captured["cmd"] = cmd
        captured["cwd"] = cwd
        captured["env"] = env
        captured["check"] = check
        return types.SimpleNamespace(returncode=0)

    monkeypatch.setattr(subprocess, "run", _fake_run)
    monkeypatch.setattr(
        sys,
        "argv",
        ["lingtu.py", "tare_explore", "--endpoint", "mujoco_live"],
    )

    main_mod.main()

    assert captured["cmd"] == [
        "bash",
        "sim/scripts/launch_mujoco_fastlio2_live.sh",
        "tare",
    ]
    assert (Path(captured["cwd"]) / "lingtu.py").exists()
    assert captured["env"]["LINGTU_PROFILE"] == "tare_explore"
    assert captured["env"]["LINGTU_ENDPOINT"] == "mujoco_live"
    assert captured["env"]["LINGTU_DATA_SOURCE"] == "mujoco_fastlio2_live"
    assert captured["env"]["LINGTU_RUNTIME_CONTRACT"] == "mujoco_fastlio2_live"
    assert captured["check"] is False
