import json
import os
import shlex
import shutil
import subprocess
from pathlib import Path

import pytest

from lingtu.assembly.config.lidar import lidar_environment
from runtime.config import RobotConfig
from runtime.utils.livox_config import build_mid360_config_dict, ensure_mid360_config_file, lidar_settings

ROOT = Path(__file__).resolve().parents[2]


@pytest.mark.skipif(shutil.which("bash") is None, reason="bash is required")
@pytest.mark.parametrize("override", [None, "1"])
def test_runtime_env_does_not_supply_sdk_binding_default(override):
    source = (ROOT / "scripts/deploy/thunder/runtime-env.sh").read_text(encoding="utf-8")
    setup = (
        "unset LINGTU_LIVOX_BIND_LIDAR_IP\n"
        "export LINGTU_CONFIG_DIR=/nonexistent/lingtu-config-test\n"
        "export LINGTU_REPO=/nonexistent/lingtu-config-test\n"
        "export LINGTU_PYTHON_ENV=/nonexistent/lingtu-config-test/python.env\n"
    )
    if override is not None:
        setup += f"export LINGTU_LIVOX_BIND_LIDAR_IP={override}\n"
    result = subprocess.run(
        ["bash", "-s"], capture_output=True,
        input=(setup + source + '\nprintf "%s" "${LINGTU_LIVOX_BIND_LIDAR_IP-unset}"\n').encode(),
    )
    assert result.returncode == 0, result.stderr.decode(errors="replace")
    assert result.stdout.decode() == (override or "unset")


def test_sdk_defaults_and_installation_overrides():
    settings = lidar_settings()
    settings["sdk"]["device_ports"]["points"] = 60000
    settings["sdk"]["host_ports"]["points"] = 60001
    settings["sdk"]["multicast_ip"] = "224.1.2.3"
    settings["sdk"]["bind_lidar_ip"] = True
    settings["sdk"]["pcl_data_type"] = 2
    robot = RobotConfig()
    robot.lidar.lidar_ip = "192.0.2.10"
    robot.lidar.host_ip = "192.0.2.11"
    robot.lidar.pitch = 0.5
    robot.raw = {"lidar": {"livox_ports": {"cmd_data_port_host": 60002}}}

    config = build_mid360_config_dict(robot, settings=settings)

    assert config["MID360"]["lidar_net_info"]["point_data_port"] == 60000
    host = config["MID360"]["host_net_info"][0]
    assert host["point_data_port"] == 60001
    assert host["cmd_data_port"] == 60002
    assert host["multicast_ip"] == "224.1.2.3"
    assert host["lidar_ip"] == ["192.0.2.10"]
    assert config["lidar_configs"][0]["pcl_data_type"] == 2
    assert not any(config["lidar_configs"][0]["extrinsic_parameter"].values())


def test_explicit_robot_config_does_not_inherit_product_snapshot(tmp_path, monkeypatch):
    robot = RobotConfig()
    robot.lidar.lidar_ip = "192.0.2.10"
    snapshot = build_mid360_config_dict(robot)
    monkeypatch.setenv("LINGTU_LIVOX_CONFIG_JSON", json.dumps(snapshot))
    robot.lidar.lidar_ip = "192.0.2.20"

    generated = ensure_mid360_config_file(robot, str(tmp_path))

    config = json.loads(Path(generated).read_text(encoding="utf-8"))
    assert config["lidar_configs"][0]["ip"] == "192.0.2.20"


@pytest.mark.skipif(shutil.which("bash") is None, reason="bash is required")
@pytest.mark.parametrize("imu_hz", [0.0, 60.0])
def test_field_launcher_passes_resolved_acquisition_values(tmp_path, imu_hz):
    def shell_path(path):
        if os.name != "nt":
            return str(path)
        return subprocess.check_output(
            ["bash", "-lc", f"wslpath -a {shlex.quote(str(path))}"],
            text=True, encoding="utf-8", errors="replace",
        ).strip()

    source = (ROOT / "scripts/deploy/thunder/run_livox_dds.sh").read_text(encoding="utf-8")
    source = source.replace("source /opt/lingtu/current/scripts/deploy/thunder/require_product_session.sh lidar", ":")
    source = source.replace("source /opt/lingtu/config/thunder-runtime-env.sh", ":")
    launcher = tmp_path / "launch.sh"
    launcher.write_text(source, encoding="utf-8", newline="\n")
    recorder = tmp_path / "record.sh"
    recorder.write_text('#!/bin/sh\nprintf "%s\\n" "$@"\n', encoding="utf-8", newline="\n")
    settings = lidar_settings()
    settings["scan_hz"], settings["imu_hz"] = 12.0, imu_hz
    environment = lidar_environment(settings, RobotConfig())
    config = tmp_path / "sdk.json"
    config.write_text(environment["LINGTU_LIVOX_CONFIG_JSON"], encoding="utf-8")
    environment.update({
        "LINGTU_LIVOX_CONFIG": shell_path(config),
        "LINGTU_LIVOX_BIN": shell_path(recorder),
        "LINGTU_CONFIG_PATH": "selected-robot.yaml",
        "LINGTU_LIVOX_NET_IFACE": "test0",
        "LINGTU_LIVOX_HOST_IP": "192.0.2.11",
        "LINGTU_LIVOX_LIDAR_IP": "192.0.2.10",
    })
    command = f"chmod +x {shlex.quote(environment['LINGTU_LIVOX_BIN'])} && env "
    command += " ".join(f"{key}={shlex.quote(value)}" for key, value in environment.items())
    command += " bash " + shlex.quote(shell_path(launcher))
    result = subprocess.run(["bash", "-lc", command], capture_output=True, text=True, encoding="utf-8", errors="replace", check=False)
    assert result.returncode == 0, result.stderr
    args = result.stdout.splitlines()
    assert args[args.index("--publish-freq") + 1] == "12"
    assert args[args.index("--imu-publish-freq") + 1] == format(imu_hz, ".15g")
    assert json.loads(config.read_text(encoding="utf-8"))["lidar_summary_info"]["lidar_type"] == 8


def test_sim_zero_imu_rate_keeps_two_samples_per_scan(tmp_path):
    import struct
    import sys

    config = tmp_path / "sdk.json"
    config.write_text("{}", encoding="utf-8")
    environment = {
        **os.environ,
        "LINGTU_LIVOX_SIM_HZ": "3",
        "LINGTU_LIVOX_SIM_FRAMES": "2",
        "LINGTU_LIVOX_SIM_IMU_PER_FRAME": "2",
    }
    result = subprocess.run(
        [sys.executable, str(ROOT / "sim/tools/livox_stream_sim.py"), str(config), "--imu-publish-freq", "0"],
        env=environment, capture_output=True, check=True, timeout=5,
    )
    header = struct.Struct("<4sB3xQIII")
    types = []
    offset = 0
    while offset < len(result.stdout):
        magic, record_type, _, _, _, payload_size = header.unpack_from(result.stdout, offset)
        assert magic == b"LTU1"
        types.append(record_type)
        offset += header.size + payload_size
    assert types == [2, 2, 1, 2, 2, 1]
