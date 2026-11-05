import json
from pathlib import Path
from shutil import copytree

import pytest
import yaml

from lingtu.assembly.compile_plan import compile_run_plan
from lingtu.assembly.config.lidar import lidar_environment
from lingtu.assembly.config.slam import slam_environment
from lingtu.assembly.graph.loader import load_runtime_graph
from runtime.config import RobotConfig
from runtime.utils.livox_config import ensure_mid360_config_file, lidar_settings


def test_component_yaml_changes_reach_native_launch(tmp_path, monkeypatch):
    source = Path(__file__).resolve().parents[3] / "config/system"
    target = tmp_path / "system"
    copytree(source, target)
    edits = {
        "maps": ("maps", "occupancy", "resolution_m", 0.1),
        "camera": ("camera", "capture", "timeout_ms", 1234),
        "lidar": ("lidar", "scan_hz", 12.0),
        "limits": ("native_nav", "braking_deceleration_mps2", 5.0),
        "local": ("parameters", "scan_planner", "smooth_weight", 2.0),
        "global": ("native_nav", "global", "snap_search_radius_cells", 13),
        "recovery": ("native_nav", "recovery", "translation_timeout_s", 2.7),
        "slam": ("slam", "runtime", "tick_hz", 40.0),
    }
    for name, keys in edits.items():
        path = target / "settings" / f"{name}.yaml"
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
        group = data
        for key in keys[:-2]:
            group = group[key]
        group[keys[-2]] = keys[-1]
        path.write_text(yaml.safe_dump(data), encoding="utf-8")

    path = target / "settings/lidar.yaml"
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    data["lidar"]["sdk"]["host_ports"]["points"] = 60001
    data["lidar"]["sdk"]["pattern_mode"] = 1
    path.write_text(yaml.safe_dump(data), encoding="utf-8")

    plan = compile_run_plan(
        "nav", "real", robot="unitree/go2", product_variant="camera",
        graph=load_runtime_graph(target),
    )
    environment = plan.native_process_environment
    assert float(environment["LINGTU_MAPD_OCCUPANCY_RESOLUTION_M"]) == 0.1
    assert float(environment["LINGTU_NAV_OCTO_BODY_CLEARANCE_ABOVE_M"]) == pytest.approx(0.3)
    assert environment["LINGTU_CAMERA_CAPTURE_TIMEOUT_MS"] == "1234"
    assert environment["LINGTU_LIVOX_SCAN_HZ"] == "12"
    assert environment["LINGTU_SLAM_TICK_HZ"] == "40"
    sdk = json.loads(environment["LINGTU_LIVOX_CONFIG_JSON"])
    assert sdk["MID360"]["host_net_info"][0]["point_data_port"] == 60001
    assert sdk["lidar_configs"][0]["pattern_mode"] == 1
    assert sdk["lidar_configs"][0]["ip"] == "192.168.123.20"
    monkeypatch.setenv("LINGTU_LIVOX_CONFIG_JSON", environment["LINGTU_LIVOX_CONFIG_JSON"])
    generated = ensure_mid360_config_file(None, str(tmp_path / "sdk"))
    assert json.loads(Path(generated).read_text(encoding="utf-8")) == sdk
    assert float(environment["LINGTU_NAV_SMOOTHER_X_DECEL_MPS2"]) == 5.0
    assert float(environment["LINGTU_NAV_SMOOTHER_Y_DECEL_MPS2"]) == 5.0
    assert plan.parameters["scan_planner.smooth_weight"] == 2.0
    assert environment["LINGTU_NAV_OCTO_SNAP_RADIUS_CELLS"] == "13"
    assert float(environment["LINGTU_NAV_RECOVERY_TRANSLATION_TIMEOUT_S"]) == 2.7


@pytest.mark.parametrize("key,value", [("scan_hz", 0), ("imu_hz", -1), ("scan_hz", True)])
def test_invalid_lidar_settings(key, value):
    settings = lidar_settings()
    settings[key] = value
    with pytest.raises(ValueError, match=f"lidar.{key}"):
        lidar_environment(settings, RobotConfig())


@pytest.mark.parametrize("key", ["tick_hz", "track_against_map_period_s"])
def test_slam_rejects_subminimum_schedule(key):
    runtime = {
        "tick_hz": 1, "log_status_s": 0, "status_json_hz": 1,
        "cloud_snapshot_hz": 0, "lidar_scan_snapshot_hz": 0,
        "track_against_map_period_s": 0, "sim": {},
    }
    runtime[key] = 0.5
    with pytest.raises(ValueError, match=rf"slam\.runtime\.{key}"):
        slam_environment({"runtime": runtime}, simulation=False)
    runtime[key] = 1 if key == "tick_hz" else 0
    environment = slam_environment({"runtime": runtime}, simulation=False)
    assert environment["LINGTU_SLAM_TICK_HZ"] == "1"
    assert environment["LINGTU_SLAM_TRACK_AGAINST_MAP_PERIOD_S"] == "0"
