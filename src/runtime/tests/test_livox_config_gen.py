import json
from pathlib import Path


def test_mid360_config_generated_from_robot_config(tmp_path, monkeypatch):
    # Avoid touching the real HOME on CI machines
    monkeypatch.setenv("HOME", str(tmp_path))

    from runtime.config import RobotConfig
    from runtime.utils.livox_config import build_mid360_config_dict, ensure_mid360_config_file

    cfg = RobotConfig()
    cfg.lidar.lidar_ip = "192.168.9.10"
    cfg.lidar.host_ip = "192.168.9.5"

    d = build_mid360_config_dict(cfg)
    host_info = d["MID360"]["host_net_info"][0]
    assert d["lidar_configs"][0]["ip"] == "192.168.9.10"
    assert host_info["host_ip"] == "192.168.9.5"
    assert host_info["lidar_ip"] == ["192.168.9.10"]
    assert host_info["point_data_port"] == 56301
    assert host_info["imu_data_port"] == 56401

    out = ensure_mid360_config_file(cfg)
    assert out.endswith("MID360_config.json")
    assert Path(out).exists()
    parsed = json.loads(Path(out).read_text(encoding="utf-8"))
    assert parsed["lidar_configs"][0]["ip"] == "192.168.9.10"
    assert parsed["MID360"]["host_net_info"][0]["host_ip"] == "192.168.9.5"
    assert parsed["MID360"]["host_net_info"][0]["lidar_ip"] == ["192.168.9.10"]

