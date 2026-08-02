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
        self.module_names: tuple[str, ...] = ()

    def build(self, transport=None) -> _FakeSystem:
        self._system.build_transport = transport
        return self._system

    def route_contract(self, *args, **kwargs):
        return self

    def require_modules(self, *names, **kwargs):
        self.module_names = tuple(dict.fromkeys((*self.module_names, *names)))
        return self


def _install_cli_harness(monkeypatch, tmp_path, system: _FakeSystem) -> dict:
    import cli.main as main_mod
    import lingtu.assembly.products as products_mod
    import lingtu.assembly.profile_builder as builder_mod

    calls = {"product": []}
    monkeypatch.delenv("LINGTU_SYSTEMD_UNIT", raising=False)
    monkeypatch.delenv("LINGTU_RUN_PLAN", raising=False)
    monkeypatch.delenv("LINGTU_RUN_PLAN_FINGERPRINT", raising=False)

    def _fake_thunder_blueprint(config=None, **overrides):
        resolved = dict(config or {})
        resolved.update(overrides)
        calls["product"].append(resolved)
        return _FakeBuilder(system)

    monkeypatch.setattr(products_mod, "thunder_blueprint", _fake_thunder_blueprint)

    def _fake_build_host_system(profile_name, host_config, *, plan=None):
        assert plan is None
        blueprint = builder_mod.blueprint_for_resolved_profile(profile_name, host_config)
        transport = builder_mod.module_transport_for_resolved_config(host_config)
        if transport is None:
            return blueprint.build()
        return blueprint.build(transport=transport)

    monkeypatch.setattr(main_mod, "_build_host_system", _fake_build_host_system)

    fake_service_manager = types.ModuleType("runtime.service_manager")
    fake_service_manager.get_service_manager = lambda: types.SimpleNamespace(_started=[])
    monkeypatch.setitem(sys.modules, "runtime.service_manager", fake_service_manager)

    monkeypatch.setattr(main_mod, "setup_logging", lambda *args, **kwargs: tmp_path)
    monkeypatch.setattr(main_mod, "preflight", lambda *args, **kwargs: None)
    monkeypatch.setattr(main_mod, "health_check", lambda _system: True)
    monkeypatch.setattr(main_mod, "print_banner", lambda *args, **kwargs: None)
    monkeypatch.setattr(main_mod, "save_run_state", lambda *args, **kwargs: None)
    monkeypatch.setattr(main_mod, "update_run_state", lambda *args, **kwargs: None)
    monkeypatch.setattr(main_mod, "clear_run_state", lambda: None)
    monkeypatch.setattr(main_mod.signal, "signal", lambda *args, **kwargs: None)
    return calls


def test_cli_shutdown_has_no_ros2_runtime_hook() -> None:
    source = Path("cli/main.py").read_text(encoding="utf-8-sig")

    assert "from runtime.ros2_context import shutdown_shared_executor" not in source
    assert "from runtime.adapters.ros2.context import shutdown_shared_executor" not in source
    assert "from lingtu.ros2_shutdown import shutdown_ros2_runtime" not in source


def test_cli_rejects_direct_start_of_product_control_managed_product(
    monkeypatch,
) -> None:
    import cli.main as main_mod

    monkeypatch.delenv("LINGTU_SYSTEMD_UNIT", raising=False)
    product = types.SimpleNamespace(
        product="nav",
        process_control="systemd",
    )

    with pytest.raises(RuntimeError, match="use scripts/lingtu"):
        main_mod._require_managed_product_entry(product)


def test_cli_has_no_residual_port_kill_control_plane() -> None:
    sources = {
        path: Path(path).read_text(encoding="utf-8-sig")
        for path in ("cli/main.py", "cli/lifecycle.py")
    }

    for path, source in sources.items():
        assert "kill_residual_ports" not in source, path
        assert "_clear_local_profile_ports" not in source, path
        assert "fuser" not in source, path


def test_managed_host_uses_run_plan_without_profile_runtime_resolution() -> None:
    import cli.main as main_mod

    config = {"command_output_mode": "endpoint_only"}
    summary = {"kind": "run_plan", "product": "nav", "env": "real"}
    plan = types.SimpleNamespace(
        product="nav",
        host_config=dict(config),
        summary=lambda: dict(summary),
    )

    resolved = main_mod._apply_runtime_process_env(
        "nav",
        config,
        types.SimpleNamespace(record=False, extra=[]),
        plan=plan,
    )

    assert resolved == summary


def test_cli_help_shows_local_profile_adapters_not_deployment_aliases(
    monkeypatch,
    capsys,
) -> None:
    import cli.main as main_mod

    monkeypatch.setattr(sys, "argv", ["lingtu.py", "--help"])

    with pytest.raises(SystemExit) as exc:
        main_mod.main()

    out = capsys.readouterr().out
    assert exc.value.code == 0
    assert "--adapter thunder_dds" not in out
    assert "--adapter legacy-field" not in out
    assert "--adapter thunder-lite" not in out
    assert "--adapter thunder-basic" not in out
    assert "lite selects the canonical thunder_lite adapter automatically" in out
    assert "--adapter mujoco_live" in out
    assert "--adapter gazebo" not in out
    assert "--adapter cmu_unity" not in out
    assert "real_s100p" not in out
    assert "s100p" not in out.lower()


@pytest.mark.parametrize(
    "adapter_name",
    ("real_s100p", "thunder-lite", "thunder-basic"),
)
def test_cli_rejects_removed_profile_adapter_alias(
    monkeypatch,
    capsys,
    adapter_name,
) -> None:
    import cli.main as main_mod

    monkeypatch.setattr(
        sys,
        "argv",
        ["lingtu.py", "runtime-spec", "sim_nav", "--adapter", adapter_name, "--json"],
    )

    with pytest.raises(SystemExit):
        main_mod.main()

    assert "unknown profile adapter" in capsys.readouterr().out


def test_lite_preflight_stays_on_lite_lifecycle_without_runtime_extra() -> None:
    import cli.main as main_mod

    main_mod.preflight(
        "lite",
        {
            "runtime_mode": "lite",
            "slam_profile": "none",
            "enable_native": False,
            "python_autonomy_backend": "simple",
            "python_path_follower_backend": "pid",
            "enable_gateway": False,
            "run_startup_checks": False,
        },
    )


@pytest.mark.parametrize(
    ("override", "expected"),
    [
        ({"slam_profile": "fastlio2"}, "slam_profile must be none"),
        ({"enable_native": True}, "enable_native must be false"),
        ({"enable_gateway": True}, "enable_gateway must be false"),
        ({"run_startup_checks": True}, "run_startup_checks must be false"),
        (
            {"enable_nav_in": True},
            "enable_nav_in must be false",
        ),
        (
            {"enable_nav_out": True},
            "enable_nav_out must be false",
        ),
        (
            {"enable_map_out": True},
            "enable_map_out must be false",
        ),
        (
            {"enable_ros2_rerun_bridge": True},
            "enable_ros2_rerun_bridge must be false",
        ),
        ({"module_transport": "zmq"}, "module_transport must be local"),
    ],
)
def test_lite_preflight_rejects_field_runtime_lifecycle_overrides(
    override,
    expected,
    capsys,
) -> None:
    import cli.main as main_mod

    cfg = {
        "runtime_mode": "lite",
        "slam_profile": "none",
        "enable_native": False,
        "enable_gateway": False,
        "run_startup_checks": False,
    }
    cfg.update(override)

    with pytest.raises(SystemExit) as exc:
        main_mod.preflight("lite", cfg)

    assert exc.value.code == 2
    output = capsys.readouterr().out
    assert "Lite runtime" in output
    assert expected in output


@pytest.mark.parametrize(
    ("args", "expected"),
    [
        (["--slam-profile", "fastlio2"], "slam_profile must be none"),
        (["--native"], "enable_native must be false"),
        (["--module-transport", "dds"], "module_transport must be local"),
        (["--adapter", "legacy-field"], "profile_adapter must be thunder_lite"),
    ],
)
def test_runtime_spec_rejects_lite_field_runtime_cli_overrides(
    args,
    expected,
    monkeypatch,
    capsys,
) -> None:
    import cli.main as main_mod

    monkeypatch.setattr(
        sys,
        "argv",
        ["lingtu.py", "runtime-spec", "lite", *args, "--json"],
    )

    with pytest.raises(SystemExit) as exc:
        main_mod.main()

    assert exc.value.code == 2
    output = capsys.readouterr().out
    assert "Lite runtime" in output
    assert expected in output


def test_fastlio2_preflight_reports_no_windows_portable_runtime(
    monkeypatch,
    capsys,
) -> None:
    import cli.runtime_extra as runtime_extra

    monkeypatch.setattr(runtime_extra.os, "name", "nt", raising=False)
    monkeypatch.setattr(runtime_extra, "_native_nav_kernel_available", lambda: True)

    runtime_extra.preflight(
        "map",
        {
            "slam_profile": "fastlio2",
            "enable_native": False,
            "enable_gateway": False,
        },
    )

    output = capsys.readouterr().out
    assert "Windows local FastLIO2 has no supported portable runtime" in output
    assert "previous portable-lio endpoint was removed" in output
    assert "ros2 not in PATH" not in output
    assert "/opt/ros/humble" not in output


def test_dds_endpoint_preflight_skips_ubuntu_ros2_guidance(
    monkeypatch,
    capsys,
) -> None:
    import shutil

    import cli.runtime_extra as runtime_extra

    monkeypatch.setattr(runtime_extra.os, "name", "posix", raising=False)
    monkeypatch.setattr(shutil, "which", lambda name: None)
    monkeypatch.setattr(runtime_extra, "_native_nav_kernel_available", lambda: True)

    runtime_extra.preflight(
        "map",
        {
            "slam_profile": "fastlio2",
            "localization_adapter": "dds_endpoint",
            "_endpoint_transport": "dds",
            "_endpoint_contract": "thunder_dds_v1",
            "enable_native": False,
            "enable_gateway": False,
        },
    )

    output = capsys.readouterr().out
    assert output == ""
    assert "ros2 not in PATH" not in output
    assert "/opt/ros/humble" not in output


def test_managed_fastlio2_preflight_keeps_ros2_guidance(
    monkeypatch,
    capsys,
) -> None:
    import shutil

    import cli.runtime_extra as runtime_extra

    monkeypatch.setattr(runtime_extra.os, "name", "posix", raising=False)
    monkeypatch.setattr(shutil, "which", lambda name: None)
    monkeypatch.setattr(runtime_extra, "_native_nav_kernel_available", lambda: True)

    runtime_extra.preflight(
        "map",
        {
            "slam_profile": "fastlio2",
            "enable_native": False,
            "enable_gateway": False,
        },
    )

    output = capsys.readouterr().out
    assert "ros2 not in PATH" in output
    assert "/opt/ros/humble" in output


def test_nav_kernel_preflight_points_to_standalone_builder(
    monkeypatch,
    capsys,
) -> None:
    import cli.runtime_extra as runtime_extra

    monkeypatch.setattr(runtime_extra, "_native_nav_kernel_available", lambda: False)

    with pytest.raises(SystemExit) as exc:
        runtime_extra.preflight(
            "nav",
            {
                "slam_profile": "none",
                "enable_native": True,
                "enable_gateway": False,
            },
        )

    assert exc.value.code == 2
    output = capsys.readouterr().out
    assert "bash scripts/build/build_nav_kernel.sh" in output
    assert "production chain will not silently fall back" in output
    assert "make build" not in output
    assert "needs ROS2" not in output


def test_octoplanner3d_preflight_points_to_standalone_builder(
    monkeypatch,
    capsys,
) -> None:
    import cli.runtime_extra as runtime_extra

    monkeypatch.setattr(runtime_extra, "_native_nav_kernel_available", lambda: True)
    monkeypatch.setattr(
        runtime_extra,
        "_octoplanner3d_runtime_errors",
        lambda _cfg: ("OctoPlanner3D C++ executable not configured",),
    )

    with pytest.raises(SystemExit) as exc:
        runtime_extra.preflight(
            "sim_nav",
            {
                "slam_profile": "none",
                "planner": "octoplanner3d",
                "enable_native": False,
                "enable_gateway": False,
                "python_autonomy_backend": "nanobind",
                "python_path_follower_backend": "nav_kernel",
            },
        )

    assert exc.value.code == 2
    output = capsys.readouterr().out
    assert "OctoPlanner3D runtime is required" in output
    assert "build_octoplanner3d.sh" in output


def test_no_repl_exits_nonzero_when_gateway_server_returns_false(monkeypatch, tmp_path):
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

    assert len(calls["product"]) == 1
    assert calls["product"][0]["robot"] == "stub"
    assert gateway._defer_server is True
    assert gateway.run_server_called is True
    assert system.started is True
    assert system.stopped is True


def test_in_process_profile_overrides_stale_runtime_env(monkeypatch, tmp_path, capsys):
    import cli.main as main_mod

    monkeypatch.setenv("LINGTU_PROFILE", "explore")
    monkeypatch.setenv("LINGTU_PROFILE_ADAPTER", "mujoco_live")
    monkeypatch.setenv("LINGTU_DATA_SOURCE", "mujoco_fastlio2_live")
    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "mujoco_fastlio2_live")
    monkeypatch.setenv("LINGTU_ENDPOINT_CONTRACT", "stale_endpoint_contract")
    monkeypatch.setenv("LINGTU_ROUTE_CONTRACT", "stale_route_contract")
    monkeypatch.setenv("LINGTU_LOCALIZATION_ADAPTER", "stale_localization_adapter")
    monkeypatch.setenv("LINGTU_ENABLE_ROBOT_DRIVER", "1")
    monkeypatch.setenv("LINGTU_COMMAND_OUTPUT_MODE", "stale_output_mode")
    monkeypatch.setenv("LINGTU_HARDWARE_CONTROL_BOUNDARY", "stale_boundary")
    monkeypatch.setenv("LINGTU_NAV_GLOBAL_PLANNER", "stale_planner")
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
    monkeypatch.setattr(sys, "argv", ["lingtu.py", "sim_nav", "--no-repl"])

    main_mod.main()

    out = capsys.readouterr().out
    assert "Runtime:  data_source=in_process_stub" in out
    assert "endpoint=in_process" not in out
    assert os.environ["LINGTU_PROFILE"] == "sim_nav"
    for stale_key in (
        "LINGTU_PROFILE_ADAPTER",
        "LINGTU_RUNTIME_CONTRACT",
        "LINGTU_ENDPOINT_CONTRACT",
        "LINGTU_ROUTE_CONTRACT",
        "LINGTU_LOCALIZATION_ADAPTER",
        "LINGTU_ENABLE_ROBOT_DRIVER",
        "LINGTU_COMMAND_OUTPUT_MODE",
        "LINGTU_HARDWARE_CONTROL_BOUNDARY",
    ):
        assert stale_key not in os.environ
    assert os.environ["LINGTU_NAV_GLOBAL_PLANNER"] == "octoplanner3d"
    assert saved_state["runtime"]["adapter"] == "in_process"
    assert saved_state["runtime"]["data_source"] == "in_process_stub"
    assert saved_state["runtime"]["runtime_contract"] is None
    assert saved_state["runtime"]["module_transport"] == "local"
    assert saved_state["runtime"]["endpoint_transport"] == "local"
    assert saved_state["runtime"]["command_sink"] == "module_graph_driver_cmd_vel"
    assert saved_state["runtime"]["validation"] == {
        "ok": True,
        "blockers": [],
        "warnings": [],
    }
    assert len(calls["product"]) == 1
    assert calls["product"][0]["robot"] == "stub"
    assert calls["product"][0]["slam_profile"] == "none"
    assert system.started is True


def test_thunder_nav_product_alias_is_rejected_by_host_cli(monkeypatch, capsys):
    import cli.main as main_mod

    monkeypatch.setattr(sys, "argv", ["lingtu.py", "thunder-nav", "--no-repl"])

    with pytest.raises(SystemExit) as exc:
        main_mod.main()

    assert exc.value.code == 1
    assert "Unknown profile 'thunder-nav'" in capsys.readouterr().out


def test_cli_module_transport_override_reaches_runtime_build(
    monkeypatch,
    tmp_path,
):
    import cli.main as main_mod
    import lingtu.assembly.profile_builder as builder_mod

    gateway = _FakeGateway(run_result=True)
    system = _FakeSystem(gateway)
    calls = _install_cli_harness(monkeypatch, tmp_path, system)
    sentinel_transport = object()

    def fake_module_transport_for_resolved_config(config):
        assert config["module_transport"] == "shm"
        return sentinel_transport

    monkeypatch.setattr(
        builder_mod,
        "module_transport_for_resolved_config",
        fake_module_transport_for_resolved_config,
    )
    monkeypatch.setattr(
        sys,
        "argv",
        ["lingtu.py", "sim_nav", "--module-transport", "shm", "--no-repl"],
    )

    main_mod.main()

    assert os.environ["LINGTU_MODULE_TRANSPORT"] == "shm"
    assert os.environ["LINGTU_ENDPOINT_TRANSPORT"] == "local"
    assert calls["product"][0]["module_transport"] == "shm"
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
        "sim/scripts/mujoco/launch_fastlio2_live.sh",
        "status",
    ]
    assert (Path(captured["cwd"]) / "lingtu.py").exists()
    assert captured["env"]["LINGTU_PROFILE"] == "sim_mujoco_live"
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
        ["lingtu.py", "sim_mujoco_live", "status"],
    )

    main_mod.main()

    output = capsys.readouterr().out
    assert (
        "Runtime:  data_source=mujoco_fastlio2_live "
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
        "Topic frames: raw_frame=lidar_link raw=lidar_link "
        "odometry=odom,map registered_cloud=body "
        "map_cloud=map,odom global_path=map,odom "
        "local_path=map,odom,body cmd_vel=body"
    ) in output
    assert (
        "Path:     sensors=/lidar/raw_frame,/imu/raw "
        "localization_map=/slam/odometry,/slam/registered_cloud,/slam/map_cloud "
        "command=mujoco_velocity_adapter"
    ) in output
    assert "Path stages: endpoint_adapter[endpoint_adapter|native_to_canonical]" in output
    assert "global_planning[lingtu_navigation_or_planner_backend|map]" in output
    assert "command_boundary[command_arbiter_to_driver|body_twist]" in output
    assert captured["cmd"] == [
        "bash",
        "sim/scripts/mujoco/launch_fastlio2_live.sh",
        "status",
    ]
    assert captured["env"]["LINGTU_PROFILE"] == "sim_mujoco_live"
    assert captured["env"]["LINGTU_PROFILE_ADAPTER"] == "mujoco_live"
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
    monkeypatch.setenv("LINGTU_PROFILE_ADAPTER", "stale_adapter")
    monkeypatch.setenv("LINGTU_DATA_SOURCE", "thunder")
    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "real")
    monkeypatch.setenv("LINGTU_COMMAND_SINK", "driver")
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
        ["lingtu.py", "sim_mujoco_live", "status"],
    )

    main_mod.main()

    output = capsys.readouterr().out
    assert "Runtime:  data_source=mujoco_fastlio2_live" in output
    assert "endpoint=mujoco_live" not in output
    assert "command_sink=mujoco_velocity_adapter simulation_only=true" in output
    assert captured["cmd"] == [
        "bash",
        "sim/scripts/mujoco/launch_fastlio2_live.sh",
        "status",
    ]
    assert captured["env"]["LINGTU_PROFILE"] == "sim_mujoco_live"
    assert captured["env"]["LINGTU_PROFILE_ADAPTER"] == "mujoco_live"
    assert captured["env"]["LINGTU_DATA_SOURCE"] == "mujoco_fastlio2_live"
    assert captured["env"]["LINGTU_ENDPOINT_TRANSPORT"] == "local"
    assert captured["env"]["LINGTU_RUNTIME_CONTRACT"] == "mujoco_fastlio2_live"
    assert captured["env"]["LINGTU_COMMAND_SINK"] == "mujoco_velocity_adapter"
    assert captured["env"]["LINGTU_SIMULATION_ONLY"] == "1"


def test_profile_adapter_launcher_accepts_trailing_action_after_options(monkeypatch):
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
        ["lingtu.py", "sim_mujoco_live", "status"],
    )

    main_mod.main()

    assert captured["cmd"] == [
        "bash",
        "sim/scripts/mujoco/launch_fastlio2_live.sh",
        "status",
    ]
    assert captured["env"]["LINGTU_PROFILE"] == "sim_mujoco_live"
    assert captured["env"]["LINGTU_PROFILE_ADAPTER"] == "mujoco_live"


@pytest.mark.sim
def test_mujoco_live_adapter_accepts_visible_demo_action(monkeypatch):
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
        ["lingtu.py", "sim_mujoco_live", "demo"],
    )

    main_mod.main()

    assert captured["cmd"] == [
        "bash",
        "sim/scripts/mujoco/launch_fastlio2_live.sh",
        "demo",
    ]
    assert captured["env"]["LINGTU_PROFILE"] == "sim_mujoco_live"
    assert captured["env"]["LINGTU_PROFILE_ADAPTER"] == "mujoco_live"


def test_switch_plan_json_separates_lifecycle_from_compiled_product(monkeypatch, capsys):
    import cli.main as main_mod

    monkeypatch.setattr(
        sys,
        "argv",
        ["lingtu.py", "switch-plan", "teleop", "nav", "--json"],
    )

    main_mod.main()

    payload = json.loads(capsys.readouterr().out)
    switch = payload["product_mode_switch"]
    assert switch["current"]["product"] == "teleop"
    assert switch["target"]["product"] == "nav"
    assert switch["same_graph_candidate"] is False
    assert switch["online_hot_switch_supported"] is False
    assert switch["required_lifecycle"] == "cold_restart"
    assert switch["target"]["native_control_mode"] == "autonomy"
    assert "product" not in switch
    assert "native_nav_config" not in switch
    run_plan = payload["run_plan"]
    native = payload["native_nav_config"]
    assert native["product"] == "nav"
    assert native["parameters"]["path_follower_max_speed_mps"] == 0.2
    assert native["parameters"]["path_follower_min_speed_mps"] == 0.08
    assert native["parameters"]["path_follower_lookahead_m"] == 0.35
    assert native["parameters"]["goal_reached_m"] == 0.1
    assert native["parameters"]["waypoint_reached_m"] == 0.2
    assert native["environment"]["LINGTU_NAV_CONFIG_FINGERPRINT"] == native["fingerprint"]
    assert run_plan["identity"]["env"] == "real"
    assert run_plan["identity"]["product"] == "nav"
    assert payload["from"]["env"] == "real"
    assert payload["from"]["product"] == "teleop"
    assert payload["to"]["product"] == "nav"
    assert "endpoint" not in payload["from"]
    assert "profile" not in payload["from"]
    assert "lingtu.product.nav.v1" in run_plan["checks"]["contracts"]


def test_switch_plan_rejects_manifest_out_escape_hatch(
    monkeypatch,
    capsys,
    tmp_path,
):
    import cli.main as main_mod

    manifest_path = tmp_path / "runtime.json"
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "lingtu.py",
            "switch-plan",
            "teleop",
            "nav",
            "--json",
            "--manifest-out",
            str(manifest_path),
        ],
    )

    with pytest.raises(SystemExit) as excinfo:
        main_mod.main()

    captured = capsys.readouterr()
    assert excinfo.value.code == 1
    assert "Usage: lingtu switch-plan" in captured.out
    assert not manifest_path.exists()


def test_switch_plan_exits_when_current_boundary_is_invalid(monkeypatch, capsys):
    import cli.main as main_mod
    import lingtu.run_plan as run_plan_mod

    calls = {"count": 0}

    def _fake_validate(_plan):
        calls["count"] += 1
        if calls["count"] == 1:
            return {
                "ok": False,
                "blockers": ["current RunPlan invalid"],
                "warnings": [],
            }
        return {"ok": True, "blockers": [], "warnings": []}

    monkeypatch.setattr(run_plan_mod, "validate_run_plan_snapshot", _fake_validate)
    monkeypatch.setattr(
        sys,
        "argv",
        ["lingtu.py", "switch-plan", "teleop", "nav"],
    )

    with pytest.raises(SystemExit) as excinfo:
        main_mod.main()

    out = capsys.readouterr().out
    assert excinfo.value.code == 2
    assert "RunPlan switch preview: FAIL" in out
    assert "Current RunPlan validation: FAIL" in out
    assert "  - current RunPlan invalid" in out
    assert "Target RunPlan validation: PASS" in out


def test_runtime_spec_prints_single_profile_boundary(monkeypatch, capsys):
    import cli.main as main_mod

    monkeypatch.setattr(
        sys,
        "argv",
        [
            "lingtu.py",
            "runtime-spec",
            "sim_mujoco_live",
            "--json",
        ],
    )

    main_mod.main()

    out = capsys.readouterr().out
    assert '"ok": true' in out
    assert '"validation": {' in out
    assert '"profile": "sim_mujoco_live"' in out
    assert '"adapter": "mujoco_live"' in out
    assert '"data_source": "mujoco_fastlio2_live"' in out
    assert '"runtime_contract": "mujoco_fastlio2_live"' in out
    assert '"command_sink": "mujoco_velocity_adapter"' in out
    assert '"frame_links": {' in out
    assert '"topic_allowed_frame_ids": {' in out
    assert '"/slam/map_cloud": [' in out
    assert '"resolved_runtime_data_flow": [' in out
    assert '"startup_gates": [' in out
    assert '"/lidar/raw_frame"' in out
    assert '"LINGTU_RUNTIME_CONTRACT": "mujoco_fastlio2_live"' in out


def test_runtime_spec_default_prints_operator_summary(monkeypatch, capsys):
    import cli.main as main_mod

    monkeypatch.setattr(
        sys,
        "argv",
        ["lingtu.py", "runtime-spec", "sim_mujoco_live"],
    )

    main_mod.main()

    out = capsys.readouterr().out
    assert "Runtime spec: PASS" in out
    assert "Local Profile: profile=sim_mujoco_live adapter=mujoco_live" in out
    assert (
        "Runtime: data_source=mujoco_fastlio2_live "
        "runtime_contract=mujoco_fastlio2_live "
        "module_transport=local endpoint_transport=local "
        "command_sink=mujoco_velocity_adapter "
        "simulation_only=true"
    ) in out
    assert "SLAM: slam_source=lingtu_fastlio2" in out
    assert "Frames: map=map odom=odom body=body lidar=lidar_link" in out
    assert "Startup gates:" in out
    assert "  map_artifact,planner_runtime,localization" in out
    assert "Topic frames:" in out
    assert "  map_cloud=map,odom" in out
    assert "Data flow:" in out
    assert "  endpoint_adapter[endpoint_adapter|native_to_canonical]" in out
    assert "Runtime env:" in out
    assert "  LINGTU_MODULE_TRANSPORT=local" in out
    assert "  LINGTU_ENDPOINT_TRANSPORT=local" in out
    assert "  LINGTU_RUNTIME_CONTRACT=mujoco_fastlio2_live" in out


def test_show_config_json_exposes_public_runtime_metadata(monkeypatch, capsys):
    import cli.main as main_mod

    monkeypatch.setattr(
        sys,
        "argv",
        ["lingtu.py", "show-config", "sim_nav", "--json"],
    )

    main_mod.main()

    payload = json.loads(capsys.readouterr().out)
    assert payload["robot"] == "stub"
    assert payload["slam_profile"] == "none"
    assert payload["enable_native"] is False
    assert payload["enable_sim_lidar"] is True
    assert payload["planner_profile"]["profile"] == "sim_nav"
    assert payload["planner"] == "octoplanner3d"


def test_runtime_spec_rejects_thunder_product_alias(monkeypatch, capsys):
    import cli.main as main_mod

    monkeypatch.setattr(
        sys,
        "argv",
        ["lingtu.py", "runtime-spec", "thunder-explore", "--adapter", "mujoco_live"],
    )

    with pytest.raises(SystemExit) as exc:
        main_mod.main()

    out = capsys.readouterr().out
    assert exc.value.code == 1
    assert "Unknown profile 'thunder-explore'" in out


def test_runtime_spec_exits_when_boundary_is_invalid(monkeypatch, capsys):
    import cli.main as main_mod
    import runtime.runtime_switch as switch_mod
    from runtime.runtime_switch import RuntimeSwitchValidation

    monkeypatch.setattr(
        switch_mod,
        "validate_runtime_switch",
        lambda _spec: RuntimeSwitchValidation(False, ("runtime boundary invalid",)),
    )
    monkeypatch.setattr(
        sys,
        "argv",
        ["lingtu.py", "runtime-spec", "sim_mujoco_live"],
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
    assert payload["runtime_data_flow_topics"]["thunder"] == [
        "/lidar/raw_frame",
        "/imu/raw",
        "/slam/odometry",
        "/slam/registered_cloud",
        "/slam/map_cloud",
        "/slam/localization_health",
        "/slam/localization_quality",
        "/nav/exploration_grid",
        "/nav/terrain_map_ext",
        "/exploration/way_point",
        "/nav/goal_pose",
        "/nav/traversable_frontiers",
        "/nav/frontier_candidate",
        "/nav/global_path",
        "/nav/way_point",
        "/nav/terrain_map",
        "/nav/traversability",
        "/nav/local_path",
        "/nav/local_planner/control_hint",
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
    assert ("  thunder[hardware] source=/lidar/raw_frame,/imu/raw normalized=/lidar/raw_frame,/imu/raw") in out
    assert "Local profile data-source bindings:" in out
    assert "  sim_nav->in_process_stub mode=pure_python_navigation_sim" in out
    assert "Artifact formats:" in out
    assert "  octomap path=octomap.ot type=octomap_full_tree frame_role=map" in out
    assert "Adapter aliases:" in out
    assert "  fastlio2 /cloud_registered->/slam/registered_cloud" in out
    assert "Adapter relays:" in out
    assert "  cmu_unity /state_estimation->/slam/odometry" in out
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
        "/lidar/raw_frame",
        "/imu/raw",
        "/slam/odometry",
        "/slam/registered_cloud",
        "/slam/map_cloud",
        "/nav/global_path",
        "/nav/local_path",
        "/nav/cmd_vel",
    ]
    assert payload["real_runtime_required_endpoint_input_topics"] == [
        "/lidar/raw_frame",
        "/imu/raw",
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
    assert '"yaml_contract": {' in out
    assert '"profile_runtime_specs": {' in out
    assert '"real_runtime_collector": {' in out
    assert '"source_frame_contracts": {' in out
    assert '"source_topic_contracts": {' in out
    assert '"src/drivers/sim/mujoco/sensors.py"' in out
    assert '"sim/scripts/mujoco/live_gate.py"' in out
    assert '"sim/scripts/saved_map_relocalize_runtime_gate.py"' in out
    assert '"scripts/monitor/feishu_monitor_bot.py"' in out
    assert '"required_real_runtime_topics": [' in out


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
    assert ("  step=1 required_when=before_any_runtime_contract_or_field_readiness_claim") in out
    assert "Checks:" in out
    assert "  yaml_contract ok=true blockers=0" in out
    assert "  runtime_validation_gates ok=true blockers=0" in out
    assert "Validation gate sequence:" in out
    assert ("  step=1 runtime_audit required_when=before_any_runtime_contract_or_field_readiness_claim") in out
    assert ("  step=2 saved_map_artifact_gate required_when=saved_map_octomap_or_occupancy_artifact_is_used") in out
    assert (
        "  step=3 real_runtime_evidence "
        "required_when=before_claiming_real_runtime_or_navigation "
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
    assert "sim/scripts/mujoco/live_gate.py" in payload["checks"]["source_frame_contracts"]["checked_files"]
    assert "src/gateway/schemas.py" in payload["checks"]["source_frame_contracts"]["checked_files"]
    assert "src/runtime/msgs/geometry.py" in payload["checks"]["source_frame_contracts"]["checked_files"]
    assert "src/runtime/msgs/nav.py" in payload["checks"]["source_frame_contracts"]["checked_files"]
    assert "src/drivers/real/lidar/module.py" in payload["checks"]["source_frame_contracts"]["checked_files"]
    assert "src/drivers/real/thunder/han_dog_module.py" in payload["checks"]["source_frame_contracts"]["checked_files"]
    assert "src/decision/modules/semantic_planner.py" in payload["checks"]["source_frame_contracts"]["checked_files"]
    assert "src/perception/perception_module.py" in payload["checks"]["source_frame_contracts"]["checked_files"]
    assert (
        "sim/scripts/saved_map_relocalize_runtime_gate.py"
        in payload["checks"]["source_topic_contracts"]["checked_files"]
    )
    assert payload["checks"]["source_topic_contracts"]["ok"] is True
    assert payload["checks"]["source_topic_contracts"]["matches"] == []
    assert "scripts/monitor/feishu_config_template.py" in payload["checks"]["source_topic_contracts"]["checked_files"]


def test_runtime_audit_source_frame_contracts_reject_direct_frame_constants(monkeypatch, tmp_path):
    import cli.runtime_audit as audit_mod

    source_root = tmp_path / "src" / "localization"
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
        "frame_id = FRAMES.odom\n"
        'CONFIG = {"planning_frame_id": "map"}\n'
        "class Bad:\n"
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
        ("src/localization",),
    )

    result = audit_mod._check_source_frame_contracts()

    assert result["ok"] is False
    assert result["checked_files"] == [
        "src/localization/bad_sim_driver.py",
        "src/localization/good.py",
    ]
    assert result["matches"] == [
        {
            "file": "src/localization/bad_sim_driver.py",
            "line": 1,
            "pattern": "direct_FRAMES_runtime_frames",
            "text": "frame_id = FRAMES.odom",
        },
        {
            "file": "src/localization/bad_sim_driver.py",
            "line": 2,
            "pattern": "hardcoded_frame_config_literal",
            "text": 'CONFIG = {"planning_frame_id": "map"}',
        },
        {
            "file": "src/localization/bad_sim_driver.py",
            "line": 4,
            "pattern": "hardcoded_frame_field_default",
            "text": 'frame_id: str = "map"',
        },
        {
            "file": "src/localization/bad_sim_driver.py",
            "line": 5,
            "pattern": "hardcoded_frame_default_argument",
            "text": 'frame_id = getattr(body, "frame_id", "map")',
        },
        {
            "file": "src/localization/bad_sim_driver.py",
            "line": 6,
            "pattern": "hardcoded_frame_or_default",
            "text": 'PAYLOAD = {"frame_id": parsed.get("frame_id") or "map"}',
        },
        {
            "file": "src/localization/bad_sim_driver.py",
            "line": 7,
            "pattern": "hardcoded_frame_id_assignment",
            "text": 'child_frame_id = "body"',
        },
        {
            "file": "src/localization/bad_sim_driver.py",
            "line": 8,
            "pattern": "hardcoded_frame_config_literal",
            "text": '_odom_frame_id = "map"',
        },
    ]
    assert (
        "source frame contract violation src/localization/bad_sim_driver.py:1 direct_FRAMES_runtime_frames"
    ) in result["blockers"]


def test_runtime_audit_source_topic_contracts_reject_direct_runtime_topics(monkeypatch, tmp_path):
    import cli.runtime_audit as audit_mod

    source_root = tmp_path / "src" / "nav"
    source_root.mkdir(parents=True)
    (source_root / "good.py").write_text(
        'TOPIC = TOPICS.goal_pose\nPATH = "src/nav/mission/navigation.py"\n',
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
    assert ("source topic contract violation src/nav/bad_nav_driver.py:1 hardcoded_canonical_runtime_topic") in result[
        "blockers"
    ]


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
            "--require-octomap",
            "--require-occupancy",
            "--expected-data-source",
            "thunder",
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
    assert "--require-octomap" in cmd
    assert "--require-occupancy" in cmd
    assert cmd[cmd.index("--expected-data-source") + 1] == "thunder"
    assert cmd[cmd.index("--expected-source-profile") + 1] == "nav"
    assert cmd[cmd.index("--expected-frame-id") + 1] == "map"
    assert cmd[cmd.index("--json-out") + 1] == str(out_path)
    assert (Path(captured["cwd"]) / "lingtu.py").exists()
    assert captured["check"] is False
    assert "--json" not in cmd


@pytest.mark.sim
def test_field_check_cli_defaults_to_simulation_mode_and_writes_json(monkeypatch, tmp_path, capsys):
    import cli.main as main_mod
    import diagnostics.field.field_check as field_check_mod

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
            "--require-octomap",
            "--require-occupancy",
            "--expected-data-source",
            "thunder",
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
        "require_octomap": True,
        "require_occupancy": True,
        "expected_data_source": "thunder",
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
            "runtime_contract": "real",
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
    assert payload["runtime_contract"] == "real"
    assert payload["transport_layers"]["module_port_bus"]["primary"] is True


def test_dataflow_cli_summary_includes_stage_and_command_closure(
    monkeypatch,
    capsys,
):
    import cli.main as main_mod

    def _fake_get(gateway_url, path, *, timeout_sec, query=None):
        return {
            "schema_version": 1,
            "runtime_contract": "real",
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


def test_doctor_cli_forwards_gateway_and_explicit_ros2_args(monkeypatch):
    import subprocess
    import types

    import cli.main as main_mod

    captured = {}

    def fake_run(cmd, **_kwargs):
        captured["cmd"] = cmd
        return types.SimpleNamespace(returncode=0)

    monkeypatch.setattr(subprocess, "run", fake_run)
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "lingtu.py",
            "doctor",
            "--gateway-url",
            "http://robot.local:5050",
            "--gateway-timeout-sec",
            "3.5",
            "--ros2",
        ],
    )

    main_mod.main()

    cmd = captured["cmd"]
    assert cmd[1:3] == ["-m", "diagnostics.field.doctor"]
    assert cmd[3:] == [
        "--gateway-url",
        "http://robot.local:5050",
        "--gateway-timeout-sec",
        "3.5",
        "--env",
        "real",
        "--ros2",
    ]


def test_doctor_cli_propagates_canonical_failure(monkeypatch):
    import subprocess
    import types

    import cli.main as main_mod

    monkeypatch.setattr(
        subprocess,
        "run",
        lambda *_args, **_kwargs: types.SimpleNamespace(returncode=7),
    )
    monkeypatch.setattr(sys, "argv", ["lingtu.py", "doctor"])

    with pytest.raises(SystemExit) as failure:
        main_mod.main()

    assert failure.value.code == 7


def test_rerun_cli_defaults_to_gateway_viewer(monkeypatch):
    import subprocess
    import types

    import cli.main as main_mod

    captured = {}

    def fake_run(cmd, **_kwargs):
        captured["cmd"] = cmd
        return types.SimpleNamespace(returncode=0)

    monkeypatch.setattr(subprocess, "run", fake_run)
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "lingtu.py",
            "rerun",
            "--gateway-url",
            "http://robot.local:5050",
            "--gateway-timeout-sec",
            "3.5",
            "--once",
        ],
    )

    main_mod.main()

    cmd = captured["cmd"]
    assert Path(cmd[1]).name == "rerun_gateway_live.py"
    assert cmd[2:] == [
        "--gateway-url",
        "http://robot.local:5050",
        "--gateway-timeout-sec",
        "3.5",
        "--once",
    ]


def test_rerun_cli_routes_explicit_ros2_to_compat_viewer(monkeypatch):
    import subprocess
    import types

    import cli.main as main_mod

    captured = {}

    def fake_run(cmd, **_kwargs):
        captured["cmd"] = cmd
        return types.SimpleNamespace(returncode=0)

    monkeypatch.setattr(subprocess, "run", fake_run)
    monkeypatch.setattr(
        sys,
        "argv",
        ["lingtu.py", "rerun", "--ros2", "--native"],
    )

    main_mod.main()

    cmd = captured["cmd"]
    assert Path(cmd[1]).name == "rerun_live.py"
    assert cmd[2:] == ["--native"]


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


def test_cli_list_exposes_host_profiles_not_field_products(
    monkeypatch,
    capsys,
):
    import cli.main as main_mod

    monkeypatch.setattr(sys, "argv", ["lingtu.py", "--list"])
    main_mod.main()
    out = capsys.readouterr().out

    assert "Available Host Profiles" in out
    assert "stub" in out
    assert "dev" in out
    assert "sim" in out
    assert "sim_nav" in out
    assert "lite" not in out
    assert " nav " not in out
    assert "tare_explore" not in out
    assert "thunder-explore" not in out
    assert "sim_mujoco_live" not in out
    assert "Use --list --all" in out

    monkeypatch.setattr(sys, "argv", ["lingtu.py", "--list", "--all"])
    main_mod.main()
    out = capsys.readouterr().out

    assert "All Host Profiles" in out
    assert "sim_mujoco_live" in out
    assert "super_lio" not in out
    assert "super_lio_relocation" not in out
    assert "lite" in out
    assert "Local Thunder hardware diagnostic" in out
    assert "teleop_avoid" not in out


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


def test_real_runtime_evidence_cli_runs_read_only_collector(monkeypatch, tmp_path, capsys):
    import subprocess

    import cli.main as main_mod

    captured = {}

    def _fake_run(cmd, *, cwd, check):
        captured["cmd"] = cmd
        captured["cwd"] = cwd
        captured["check"] = check
        return types.SimpleNamespace(returncode=0)

    out_path = tmp_path / "real_runtime" / "report.json"
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
    assert cmd[cmd.index("--collector") + 1] == "gateway"
    assert cmd[cmd.index("--gateway-url") + 1] == "http://127.0.0.1:5050"
    assert cmd[cmd.index("--gateway-timeout-sec") + 1] == "2.0"
    assert cmd[cmd.index("--duration-sec") + 1] == "3.5"
    assert cmd[cmd.index("--min-motion-m") + 1] == "0.2"
    assert cmd[cmd.index("--min-cmd-vel-norm") + 1] == "0.07"
    assert cmd[cmd.index("--expected-contract") + 1] == "real"
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


def test_real_runtime_evidence_cli_prints_validation_blockers(monkeypatch, tmp_path, capsys):
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
                        "command_sink": "driver",
                    },
                    "runtime_contract": {"name": "real", "ok": False},
                    "runtime_evidence": {
                        "ok": False,
                        "validation_gate": {
                            "acceptance_step": 3,
                            "required_when": ("before_claiming_real_runtime_or_navigation"),
                            "requires_prior_gates": ["runtime_audit"],
                            "conditional_prior_gates": [
                                "saved_map_artifact_gate when saved map, tomogram, occupancy, or PCT artifact is used"
                            ],
                            "proves": [
                                "observed_real_runtime_contract",
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
                                "missing_outputs": ["driver"],
                                "missing_signals": ["hardware_command_route"],
                                "owner": "command_arbiter_to_driver",
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

    out_path = tmp_path / "real_runtime" / "report.json"
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
    assert "Runtime contract: name=real ok=false" in out
    assert "Validation gate:" in out
    assert (
        "  step=3 required_when=before_claiming_real_runtime_or_navigation prior=runtime_audit"
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
    assert "  command_boundary[command_arbiter_to_driver|body_twist] ok=false" in out


@pytest.mark.sim
def test_mujoco_live_adapter_accepts_pct_moving_obstacle_action(monkeypatch):
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
        "sim/scripts/mujoco/launch_fastlio2_live.sh",
        "pct-moving-obstacle",
    ]
    assert captured["env"]["LINGTU_PROFILE"] == "sim_mujoco_live"
    assert captured["env"]["LINGTU_RUNTIME_CONTRACT"] == "mujoco_fastlio2_live"

@pytest.mark.sim
def test_mujoco_live_adapter_accepts_inspection_video_action(monkeypatch):
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
        "sim/scripts/mujoco/launch_fastlio2_live.sh",
        "inspection-moving-obstacle-video",
    ]
    assert captured["env"]["LINGTU_PROFILE"] == "sim_mujoco_live"
    assert captured["env"]["LINGTU_RUNTIME_CONTRACT"] == "mujoco_fastlio2_live"
